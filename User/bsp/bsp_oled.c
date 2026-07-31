#include "bsp_oled.h"

#include "i2c.h"
#include "main.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define OLED_ALL_PAGES_MASK          ((uint8_t)0xFFU)
#define OLED_NO_ACTIVE_PAGE          ((uint8_t)0xFFU)
#define OLED_CONTROL_COMMAND         ((uint8_t)0x00U)
#define OLED_CONTROL_DATA            ((uint8_t)0x40U)

typedef enum
{
    OLED_PHASE_IDLE = 0,
    OLED_PHASE_INIT_WAIT,
    OLED_PHASE_PAGE_COMMAND_WAIT,
    OLED_PHASE_PAGE_DATA_START,
    OLED_PHASE_PAGE_DATA_WAIT
} OledPhase_t;

static uint8_t s_framebuffer[BSP_OLED_PAGE_COUNT][BSP_OLED_WIDTH];
static uint8_t s_dirty_mask = 0U;

static bool s_initialized = false;
static bool s_online = false;
static bool s_transfer_busy = false;
static bool s_ever_connected = false;
static bool s_active_page_dirty_again = false;

static volatile bool s_tx_complete_pending = false;
static volatile bool s_error_pending = false;
static volatile uint32_t s_callback_error = 0U;

static BspOledState_t s_state = BSP_OLED_STATE_UNINITIALIZED;
static OledPhase_t s_phase = OLED_PHASE_IDLE;

static uint8_t s_active_page = OLED_NO_ACTIVE_PAGE;
static uint8_t s_active_offset = 0U;

static uint8_t s_tx_buffer[1U + BSP_OLED_TRANSFER_CHUNK];

static uint32_t s_state_start_ms = 0U;
static uint32_t s_transfer_start_ms = 0U;
static uint32_t s_next_retry_ms = 0U;
static uint32_t s_last_success_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;

static uint32_t s_error_count = 0U;
static uint32_t s_disconnect_count = 0U;
static uint32_t s_reconnect_count = 0U;
static uint32_t s_retry_count = 0U;
static uint32_t s_hal_busy_count = 0U;
static uint32_t s_transfer_count = 0U;
static uint32_t s_last_error = 0U;

static uint32_t Oled_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Oled_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static bool Oled_Elapsed(
    uint32_t now_ms,
    uint32_t start_ms,
    uint32_t duration_ms)
{
    return ((uint32_t)(now_ms - start_ms) >= duration_ms);
}

static bool Oled_TimeReached(
    uint32_t now_ms,
    uint32_t target_ms)
{
    return ((int32_t)(now_ms - target_ms) >= 0);
}

static void Oled_BusDelay(void)
{
    volatile uint32_t count;

    for (count = 0U; count < 256U; count++)
    {
        __NOP();
    }
}

static bool Oled_RecoverHardwareBus(void)
{
    GPIO_InitTypeDef gpio = {0};

    /*
     * 先关闭I2C外设，再暂时把PB6/PB7变成开漏GPIO。
     * 输出高表示释放总线，外部上拉电阻将线路拉高。
     */
    (void)HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = OLED_SCL_Pin | OLED_SDA_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(
        GPIOB,
        OLED_SCL_Pin | OLED_SDA_Pin,
        GPIO_PIN_SET);
    Oled_BusDelay();

    for (uint8_t pulse = 0U; pulse < 9U; pulse++)
    {
        HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin, GPIO_PIN_RESET);
        Oled_BusDelay();
        HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin, GPIO_PIN_SET);
        Oled_BusDelay();
    }

    /* 产生一次STOP：SDA低，SCL高，再释放SDA。 */
    HAL_GPIO_WritePin(GPIOB, OLED_SDA_Pin, GPIO_PIN_RESET);
    Oled_BusDelay();
    HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin, GPIO_PIN_SET);
    Oled_BusDelay();
    HAL_GPIO_WritePin(GPIOB, OLED_SDA_Pin, GPIO_PIN_SET);
    Oled_BusDelay();

    /*
     * HAL_I2C_Init会重新调用HAL_I2C_MspInit，
     * 恢复PB6/PB7的AF4开漏配置。
     */
    return (HAL_I2C_Init(&hi2c1) == HAL_OK);
}

static void Oled_MarkPageDirty(uint8_t page)
{
    if (page >= BSP_OLED_PAGE_COUNT)
    {
        return;
    }

    s_dirty_mask |= (uint8_t)(1U << page);

    if ((s_active_page == page) &&
        ((s_phase == OLED_PHASE_PAGE_COMMAND_WAIT) ||
         (s_phase == OLED_PHASE_PAGE_DATA_START) ||
         (s_phase == OLED_PHASE_PAGE_DATA_WAIT)))
    {
        s_active_page_dirty_again = true;
    }
}

static void Oled_ResetActivePage(void)
{
    s_active_page = OLED_NO_ACTIVE_PAGE;
    s_active_offset = 0U;
    s_active_page_dirty_again = false;
}

static void Oled_MarkOffline(
    uint32_t now_ms,
    uint32_t error_code)
{
    if (s_online)
    {
        s_disconnect_count++;
    }

    s_online = false;
    s_transfer_busy = false;
    s_tx_complete_pending = false;
    s_error_pending = false;

    s_state = BSP_OLED_STATE_RETRY_WAIT;
    s_phase = OLED_PHASE_IDLE;
    s_next_retry_ms = now_ms + BSP_OLED_RETRY_INTERVAL_MS;

    s_dirty_mask = OLED_ALL_PAGES_MASK;
    Oled_ResetActivePage();

    s_error_count++;
    s_last_error = error_code;
}

static bool Oled_StartTransfer(
    const uint8_t *data,
    uint16_t length,
    OledPhase_t wait_phase,
    uint32_t now_ms)
{
    HAL_StatusTypeDef hal_status;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    s_tx_complete_pending = false;
    s_error_pending = false;

    hal_status = HAL_I2C_Master_Transmit_IT(
        &hi2c1,
        BSP_OLED_I2C_ADDRESS_HAL,
        (uint8_t *)data,
        length);

    if (hal_status == HAL_OK)
    {
        s_transfer_busy = true;
        s_phase = wait_phase;
        s_transfer_start_ms = now_ms;
        s_transfer_count++;
        return true;
    }

    if (hal_status == HAL_BUSY)
    {
        s_hal_busy_count++;
        Oled_MarkOffline(now_ms, HAL_I2C_ERROR_NONE);
        return false;
    }

    Oled_MarkOffline(now_ms, hi2c1.ErrorCode);
    return false;
}

static bool Oled_StartControllerInit(uint32_t now_ms)
{
    static const uint8_t commands[] =
    {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
        0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
    };

    s_tx_buffer[0] = OLED_CONTROL_COMMAND;
    memcpy(&s_tx_buffer[1], commands, sizeof(commands));

    s_state = BSP_OLED_STATE_INITIALIZING;

    return Oled_StartTransfer(
        s_tx_buffer,
        (uint16_t)(sizeof(commands) + 1U),
        OLED_PHASE_INIT_WAIT,
        now_ms);
}

static bool Oled_StartPageCommand(uint32_t now_ms)
{
    s_tx_buffer[0] = OLED_CONTROL_COMMAND;
    s_tx_buffer[1] = (uint8_t)(0xB0U | s_active_page);
    s_tx_buffer[2] =
        (uint8_t)(0x10U | (s_active_offset >> 4U));
    s_tx_buffer[3] =
        (uint8_t)(s_active_offset & 0x0FU);

    return Oled_StartTransfer(
        s_tx_buffer,
        4U,
        OLED_PHASE_PAGE_COMMAND_WAIT,
        now_ms);
}

static bool Oled_StartPageData(uint32_t now_ms)
{
    uint8_t remaining;
    uint8_t count;

    remaining =
        (uint8_t)(BSP_OLED_WIDTH - s_active_offset);

    count = (remaining > BSP_OLED_TRANSFER_CHUNK) ?
        BSP_OLED_TRANSFER_CHUNK :
        remaining;

    s_tx_buffer[0] = OLED_CONTROL_DATA;
    memcpy(
        &s_tx_buffer[1],
        &s_framebuffer[s_active_page][s_active_offset],
        count);

    return Oled_StartTransfer(
        s_tx_buffer,
        (uint16_t)(count + 1U),
        OLED_PHASE_PAGE_DATA_WAIT,
        now_ms);
}

static void Oled_SelectNextDirtyPage(void)
{
    uint8_t page;

    if (s_active_page != OLED_NO_ACTIVE_PAGE)
    {
        return;
    }

    for (page = 0U; page < BSP_OLED_PAGE_COUNT; page++)
    {
        if ((s_dirty_mask & (uint8_t)(1U << page)) != 0U)
        {
            s_active_page = page;
            s_active_offset = 0U;
            s_active_page_dirty_again = false;
            return;
        }
    }
}

static void Oled_HandleTransferComplete(uint32_t now_ms)
{
    s_transfer_busy = false;
    s_tx_complete_pending = false;
    s_last_success_ms = now_ms;

    switch (s_phase)
    {
        case OLED_PHASE_INIT_WAIT:
            s_online = true;
            s_state = BSP_OLED_STATE_ONLINE;
            s_phase = OLED_PHASE_IDLE;
            s_dirty_mask = OLED_ALL_PAGES_MASK;
            s_last_heartbeat_ms = now_ms;
            Oled_ResetActivePage();

            if (s_ever_connected)
            {
                s_reconnect_count++;
            }
            s_ever_connected = true;
            break;

        case OLED_PHASE_PAGE_COMMAND_WAIT:
            s_phase = OLED_PHASE_PAGE_DATA_START;
            break;

        case OLED_PHASE_PAGE_DATA_WAIT:
            s_active_offset =
                (uint8_t)(s_active_offset +
                          BSP_OLED_TRANSFER_CHUNK);

            if (s_active_offset >= BSP_OLED_WIDTH)
            {
                if (!s_active_page_dirty_again)
                {
                    s_dirty_mask &=
                        (uint8_t)~(1U << s_active_page);
                }

                Oled_ResetActivePage();
            }

            s_phase = OLED_PHASE_IDLE;
            break;

        case OLED_PHASE_IDLE:
        case OLED_PHASE_PAGE_DATA_START:
        default:
            s_phase = OLED_PHASE_IDLE;
            break;
    }
}

static const uint8_t *Oled_GetGlyph(char character)
{
    /*
     * 5x7 ASCII，按竖列存放，bit0为顶部像素。
     * 索引：
     * 0-9，A-Z，空格，冒号，减号，点，问号，
     * >，+，/，=
     */
    static const uint8_t glyphs[][5] =
    {
        {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
        {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},
        {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},
        {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},
        {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},
        {0x06,0x49,0x49,0x29,0x1E}, /* 9 */

        {0x7E,0x11,0x11,0x11,0x7E}, /* A */
        {0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22},
        {0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41},
        {0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A},
        {0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00},
        {0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41},
        {0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F},
        {0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E},
        {0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E},
        {0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31},
        {0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F},
        {0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F},
        {0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07},
        {0x61,0x51,0x49,0x45,0x43}, /* Z */

        {0x00,0x00,0x00,0x00,0x00}, /* space */
        {0x00,0x36,0x36,0x00,0x00}, /* : */
        {0x08,0x08,0x08,0x08,0x08}, /* - */
        {0x00,0x60,0x60,0x00,0x00}, /* . */
        {0x02,0x01,0x51,0x09,0x06}, /* ? */
        {0x00,0x41,0x22,0x14,0x08}, /* > */
        {0x08,0x08,0x3E,0x08,0x08}, /* + */
        {0x20,0x10,0x08,0x04,0x02}, /* / */
        {0x14,0x14,0x14,0x14,0x14}  /* = */
    };

    uint8_t index;

    if ((character >= 'a') && (character <= 'z'))
    {
        character =
            (char)(character - 'a' + 'A');
    }

    if ((character >= '0') && (character <= '9'))
    {
        index = (uint8_t)(character - '0');
    }
    else if ((character >= 'A') && (character <= 'Z'))
    {
        index =
            (uint8_t)(10U + (uint8_t)(character - 'A'));
    }
    else if (character == ' ')
    {
        index = 36U;
    }
    else if (character == ':')
    {
        index = 37U;
    }
    else if (character == '-')
    {
        index = 38U;
    }
    else if (character == '.')
    {
        index = 39U;
    }
    else if (character == '?')
    {
        index = 40U;
    }
    else if (character == '>')
    {
        index = 41U;
    }
    else if (character == '+')
    {
        index = 42U;
    }
    else if (character == '/')
    {
        index = 43U;
    }
    else if (character == '=')
    {
        index = 44U;
    }
    else
    {
        index = 40U;
    }

    return glyphs[index];
}

bool BSP_Oled_Init(void)
{
    uint32_t primask;
    uint32_t now_ms = HAL_GetTick();

    if ((hi2c1.Instance != I2C1) ||
        (hi2c1.Init.ClockSpeed != 400000U))
    {
        return false;
    }

    primask = Oled_EnterCritical();

    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    s_dirty_mask = OLED_ALL_PAGES_MASK;

    s_initialized = true;
    s_online = false;
    s_transfer_busy = false;
    s_ever_connected = false;
    s_active_page_dirty_again = false;

    s_tx_complete_pending = false;
    s_error_pending = false;
    s_callback_error = 0U;

    s_state = BSP_OLED_STATE_POWER_WAIT;
    s_phase = OLED_PHASE_IDLE;

    Oled_ResetActivePage();

    s_state_start_ms = now_ms;
    s_transfer_start_ms = now_ms;
    s_next_retry_ms = 0U;
    s_last_success_ms = 0U;
    s_last_heartbeat_ms = now_ms;

    s_error_count = 0U;
    s_disconnect_count = 0U;
    s_reconnect_count = 0U;
    s_retry_count = 0U;
    s_hal_busy_count = 0U;
    s_transfer_count = 0U;
    s_last_error = 0U;

    Oled_ExitCritical(primask);

    return true;
}

void BSP_Oled_Process(void)
{
    uint32_t now_ms;
    bool error_pending;
    bool tx_complete_pending;
    uint32_t callback_error;
    uint32_t primask;

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    primask = Oled_EnterCritical();
    error_pending = s_error_pending;
    tx_complete_pending = s_tx_complete_pending;
    callback_error = s_callback_error;

    if (error_pending)
    {
        s_error_pending = false;
    }
    Oled_ExitCritical(primask);

    if (error_pending)
    {
        Oled_MarkOffline(now_ms, callback_error);
        return;
    }

    if (s_transfer_busy)
    {
        if (tx_complete_pending)
        {
            Oled_HandleTransferComplete(now_ms);
        }
        else if (Oled_Elapsed(
                     now_ms,
                     s_transfer_start_ms,
                     BSP_OLED_TRANSFER_TIMEOUT_MS))
        {
            Oled_MarkOffline(
                now_ms,
                (uint32_t)HAL_TIMEOUT);
        }
        else
        {
            return;
        }
    }

    switch (s_state)
    {
        case BSP_OLED_STATE_POWER_WAIT:
            if (Oled_Elapsed(
                    now_ms,
                    s_state_start_ms,
                    BSP_OLED_POWER_STABLE_MS))
            {
                if (!Oled_RecoverHardwareBus())
                {
                    Oled_MarkOffline(
                        now_ms,
                        hi2c1.ErrorCode);
                    return;
                }

                (void)Oled_StartControllerInit(now_ms);
            }
            break;

        case BSP_OLED_STATE_RETRY_WAIT:
            if (Oled_TimeReached(
                    now_ms,
                    s_next_retry_ms))
            {
                s_retry_count++;

                if (!Oled_RecoverHardwareBus())
                {
                    Oled_MarkOffline(
                        now_ms,
                        hi2c1.ErrorCode);
                    return;
                }

                (void)Oled_StartControllerInit(now_ms);
            }
            break;

        case BSP_OLED_STATE_INITIALIZING:
            break;

        case BSP_OLED_STATE_ONLINE:
            if (!s_online)
            {
                Oled_MarkOffline(
                    now_ms,
                    HAL_I2C_ERROR_NONE);
                return;
            }

            if (s_phase == OLED_PHASE_PAGE_DATA_START)
            {
                (void)Oled_StartPageData(now_ms);
                return;
            }

            if ((s_dirty_mask == 0U) &&
                Oled_Elapsed(
                    now_ms,
                    s_last_heartbeat_ms,
                    BSP_OLED_HEARTBEAT_INTERVAL_MS))
            {
                /*
                 * 每秒重发第0页作为在线探测。
                 * OLED静态画面时拔线也能被发现并进入重连。
                 */
                Oled_MarkPageDirty(0U);
                s_last_heartbeat_ms = now_ms;
            }

            Oled_SelectNextDirtyPage();

            if (s_active_page != OLED_NO_ACTIVE_PAGE)
            {
                (void)Oled_StartPageCommand(now_ms);
            }
            break;

        case BSP_OLED_STATE_UNINITIALIZED:
        default:
            break;
    }
}

void BSP_Oled_Clear(void)
{
    memset(s_framebuffer, 0, sizeof(s_framebuffer));
    s_dirty_mask = OLED_ALL_PAGES_MASK;

    if (s_active_page != OLED_NO_ACTIVE_PAGE)
    {
        s_active_page_dirty_again = true;
    }
}

void BSP_Oled_ClearPage(uint8_t page)
{
    if (page >= BSP_OLED_PAGE_COUNT)
    {
        return;
    }

    memset(
        s_framebuffer[page],
        0,
        BSP_OLED_WIDTH);

    Oled_MarkPageDirty(page);
}

void BSP_Oled_DrawChar(
    uint8_t x,
    uint8_t page,
    char character)
{
    const uint8_t *glyph;
    uint8_t column;

    if ((page >= BSP_OLED_PAGE_COUNT) ||
        (x > (BSP_OLED_WIDTH - 6U)))
    {
        return;
    }

    glyph = Oled_GetGlyph(character);

    for (column = 0U; column < 5U; column++)
    {
        s_framebuffer[page][x + column] =
            glyph[column];
    }

    s_framebuffer[page][x + 5U] = 0U;
    Oled_MarkPageDirty(page);
}

void BSP_Oled_DrawString(
    uint8_t x,
    uint8_t page,
    const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while ((*text != '\0') &&
           (x <= (BSP_OLED_WIDTH - 6U)))
    {
        BSP_Oled_DrawChar(
            x,
            page,
            *text);

        text++;
        x = (uint8_t)(x + 6U);
    }
}

void BSP_Oled_DrawU32(
    uint8_t x,
    uint8_t page,
    uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count] =
            (char)('0' + (value % 10U));
        count++;
        value /= 10U;
    }
    while ((value != 0U) &&
           (count < (uint8_t)sizeof(digits)));

    while ((count != 0U) &&
           (x <= (BSP_OLED_WIDTH - 6U)))
    {
        count--;
        BSP_Oled_DrawChar(
            x,
            page,
            digits[count]);
        x = (uint8_t)(x + 6U);
    }
}

void BSP_Oled_DrawI32(
    uint8_t x,
    uint8_t page,
    int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        BSP_Oled_DrawChar(x, page, '-');
        x = (uint8_t)(x + 6U);

        magnitude =
            (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    BSP_Oled_DrawU32(x, page, magnitude);
}

void BSP_Oled_SetPixel(uint8_t x, uint8_t y, bool on)
{
    uint8_t page;
    uint8_t bit;

    if ((x >= BSP_OLED_WIDTH) ||
        (y >= BSP_OLED_HEIGHT))
    {
        return;
    }

    page = y / 8U;
    bit = y % 8U;

    if (on)
    {
        s_framebuffer[page][x] |= (uint8_t)(1U << bit);
    }
    else
    {
        s_framebuffer[page][x] &= (uint8_t)(~(1U << bit));
    }

    s_dirty_mask |= (uint8_t)(1U << page);
}

void BSP_Oled_FillRect(
    uint8_t x,
    uint8_t y,
    uint8_t w,
    uint8_t h,
    bool on)
{
    uint8_t col;
    uint8_t row;

    if ((x >= BSP_OLED_WIDTH) ||
        (y >= BSP_OLED_HEIGHT))
    {
        return;
    }

    if ((uint16_t)x + w > BSP_OLED_WIDTH)
    {
        w = (uint8_t)(BSP_OLED_WIDTH - x);
    }

    if ((uint16_t)y + h > BSP_OLED_HEIGHT)
    {
        h = (uint8_t)(BSP_OLED_HEIGHT - y);
    }

    for (row = 0U; row < h; row++)
    {
        for (col = 0U; col < w; col++)
        {
            BSP_Oled_SetPixel(
                (uint8_t)(x + col),
                (uint8_t)(y + row),
                on);
        }
    }
}

void BSP_Oled_MarkPagesDirty(
    uint8_t first_page,
    uint8_t page_count)
{
    uint8_t page;
    uint8_t end_page;

    if ((first_page >= BSP_OLED_PAGE_COUNT) ||
        (page_count == 0U))
    {
        return;
    }

    end_page =
        (uint8_t)(first_page + page_count);

    if ((end_page > BSP_OLED_PAGE_COUNT) ||
        (end_page < first_page))
    {
        end_page = BSP_OLED_PAGE_COUNT;
    }

    for (page = first_page;
         page < end_page;
         page++)
    {
        Oled_MarkPageDirty(page);
    }
}

bool BSP_Oled_IsInitialized(void)
{
    return s_initialized;
}

bool BSP_Oled_IsOnline(void)
{
    return s_initialized && s_online;
}

bool BSP_Oled_GetStatus(BspOledStatus_t *status)
{
    if ((!s_initialized) || (status == NULL))
    {
        return false;
    }

    status->initialized = s_initialized;
    status->online = s_online;
    status->transfer_busy = s_transfer_busy;
    status->state = s_state;

    status->dirty_mask = s_dirty_mask;
    status->active_page = s_active_page;
    status->active_offset = s_active_offset;

    status->error_count = s_error_count;
    status->disconnect_count = s_disconnect_count;
    status->reconnect_count = s_reconnect_count;
    status->retry_count = s_retry_count;
    status->hal_busy_count = s_hal_busy_count;
    status->transfer_count = s_transfer_count;
    status->last_error = s_last_error;

    status->last_success_ms = s_last_success_ms;
    status->next_retry_ms = s_next_retry_ms;

    return true;
}

void BSP_Oled_ForceReconnect(void)
{
    if (!s_initialized)
    {
        return;
    }

    Oled_MarkOffline(
        HAL_GetTick(),
        HAL_I2C_ERROR_NONE);

    s_next_retry_ms = HAL_GetTick();
}

const char *BSP_Oled_StateName(BspOledState_t state)
{
    switch (state)
    {
        case BSP_OLED_STATE_UNINITIALIZED:
            return "UNINIT";

        case BSP_OLED_STATE_POWER_WAIT:
            return "POWER";

        case BSP_OLED_STATE_INITIALIZING:
            return "INIT";

        case BSP_OLED_STATE_ONLINE:
            return "ONLINE";

        case BSP_OLED_STATE_RETRY_WAIT:
            return "RETRY";

        default:
            return "UNKNOWN";
    }
}

void BSP_Oled_TxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c == NULL) ||
        (hi2c->Instance != I2C1))
    {
        return;
    }

    s_tx_complete_pending = true;
}

void BSP_Oled_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c == NULL) ||
        (hi2c->Instance != I2C1))
    {
        return;
    }

    s_callback_error = hi2c->ErrorCode;
    s_error_pending = true;
}
