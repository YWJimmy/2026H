#include "bsp_line_uart.h"

#include "usart.h"

#include <stddef.h>

#define LINE_UART_MODE_MANUAL_COMMAND      ((uint8_t)0x00U)
#define LINE_UART_QUERY_DIGITAL_COMMAND    ((uint8_t)0x01U)

typedef enum
{
    LINE_UART_STATE_STOPPED = 0,
    LINE_UART_STATE_POWER_WAIT,
    LINE_UART_STATE_MODE_TX_WAIT,
    LINE_UART_STATE_MODE_SETTLE,
    LINE_UART_STATE_READY,
    LINE_UART_STATE_RESPONSE_WAIT
} LineUartState_t;


static volatile bool s_initialized = false;
static volatile bool s_mode_command_sent = false;
static volatile bool s_tx_complete = false;
static volatile bool s_rx_complete = false;
static volatile bool s_uart_error_pending = false;

static volatile LineUartState_t s_state = LINE_UART_STATE_STOPPED;
static volatile BspLineUartStatus_t s_status =
    BSP_LINE_UART_STATUS_UNINITIALIZED;

static uint8_t s_tx_byte = 0U;
static uint8_t s_rx_byte = 0U;
static uint8_t s_latest_raw_state = 0U;

static uint32_t s_sequence = 0U;
static uint32_t s_latest_timestamp_ms = 0U;
static uint32_t s_init_timestamp_ms = 0U;
static uint32_t s_state_timestamp_ms = 0U;
static uint32_t s_query_timestamp_ms = 0U;
static uint32_t s_next_query_ms = 0U;

static uint32_t s_query_count = 0U;
static uint32_t s_response_count = 0U;
static uint32_t s_timeout_count = 0U;
static uint32_t s_uart_error_count = 0U;
static uint32_t s_hal_busy_count = 0U;
static uint32_t s_last_uart_error = 0U;

static uint32_t LineUart_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void LineUart_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static bool LineUart_TimeReached(uint32_t now_ms, uint32_t target_ms)
{
    return ((int32_t)(now_ms - target_ms) >= 0);
}

static bool LineUart_Elapsed(uint32_t now_ms,
                             uint32_t start_ms,
                             uint32_t duration_ms)
{
    return ((uint32_t)(now_ms - start_ms) >= duration_ms);
}

static void LineUart_ResetSoftwareState(uint32_t now_ms)
{
    s_mode_command_sent = false;
    s_tx_complete = false;
    s_rx_complete = false;
    s_uart_error_pending = false;

    s_state = LINE_UART_STATE_POWER_WAIT;
    s_status = BSP_LINE_UART_STATUS_POWER_WAIT;

    s_tx_byte = 0U;
    s_rx_byte = 0U;
    s_latest_raw_state = 0U;

    s_sequence = 0U;
    s_latest_timestamp_ms = 0U;
    s_init_timestamp_ms = now_ms;
    s_state_timestamp_ms = now_ms;
    s_query_timestamp_ms = now_ms;
    s_next_query_ms = now_ms;

    s_query_count = 0U;
    s_response_count = 0U;
    s_timeout_count = 0U;
    s_uart_error_count = 0U;
    s_hal_busy_count = 0U;
    s_last_uart_error = 0U;
}

static void LineUart_EnterReady(uint32_t now_ms)
{
    s_state = LINE_UART_STATE_READY;
    s_state_timestamp_ms = now_ms;
    s_next_query_ms = now_ms;
    s_status = BSP_LINE_UART_STATUS_READY;
}

static void LineUart_HandlePendingError(uint32_t now_ms)
{
    bool pending;
    uint32_t primask;

    primask = LineUart_EnterCritical();
    pending = s_uart_error_pending;
    s_uart_error_pending = false;
    LineUart_ExitCritical(primask);

    if (!pending)
    {
        return;
    }

    (void)HAL_UART_Abort(&huart4);
    s_tx_complete = false;
    s_rx_complete = false;

    if (s_mode_command_sent)
    {
        s_state = LINE_UART_STATE_READY;
        s_next_query_ms = now_ms + 2U;
    }
    else
    {
        /* 模式命令未确认发送完成，延迟 10 ms 后重新发送。 */
        s_state = LINE_UART_STATE_POWER_WAIT;
        s_init_timestamp_ms =
            now_ms - BSP_LINE_UART_POWER_STABLE_MS + 10U;
    }
}

static void LineUart_TrySendMode(uint32_t now_ms)
{
    HAL_StatusTypeDef status;

    s_tx_byte = LINE_UART_MODE_MANUAL_COMMAND;
    s_tx_complete = false;
    s_state = LINE_UART_STATE_MODE_TX_WAIT;
    s_state_timestamp_ms = now_ms;
    s_status = BSP_LINE_UART_STATUS_MODE_SENDING;

    status = HAL_UART_Transmit_IT(&huart4, &s_tx_byte, 1U);
    if (status != HAL_OK)
    {
        s_hal_busy_count++;
            s_state = LINE_UART_STATE_POWER_WAIT;
        s_init_timestamp_ms =
            now_ms - BSP_LINE_UART_POWER_STABLE_MS + 1U;
        s_status = BSP_LINE_UART_STATUS_HAL_BUSY;
    }
}

static void LineUart_TryStartQuery(uint32_t now_ms)
{
    HAL_StatusTypeDef status;

    s_rx_byte = 0U;
    s_rx_complete = false;
    s_tx_complete = false;

    /* 先挂接接收，再发送查询，避免遗漏极短响应。 */
    status = HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
    if (status != HAL_OK)
    {
        s_hal_busy_count++;
        s_next_query_ms = now_ms + 1U;
        s_status = BSP_LINE_UART_STATUS_HAL_BUSY;
        return;
    }

    s_tx_byte = LINE_UART_QUERY_DIGITAL_COMMAND;
    s_state = LINE_UART_STATE_RESPONSE_WAIT;
    s_query_timestamp_ms = now_ms;
    s_query_count++;
    s_status = BSP_LINE_UART_STATUS_WAIT_RESPONSE;

    status = HAL_UART_Transmit_IT(&huart4, &s_tx_byte, 1U);
    if (status != HAL_OK)
    {
        s_hal_busy_count++;
        (void)HAL_UART_AbortReceive(&huart4);
            s_state = LINE_UART_STATE_READY;
        s_next_query_ms = now_ms + 1U;
        s_status = BSP_LINE_UART_STATUS_HAL_BUSY;
    }
}

bool BSP_LineUart_Init(void)
{
    uint32_t primask;
    uint32_t now_ms = HAL_GetTick();

    if (huart4.Instance != UART4)
    {
        return false;
    }

    (void)HAL_UART_Abort(&huart4);

    primask = LineUart_EnterCritical();
    LineUart_ResetSoftwareState(now_ms);
    s_initialized = true;
    LineUart_ExitCritical(primask);

    return true;
}

void BSP_LineUart_Process(void)
{
    uint32_t now_ms;

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();
    LineUart_HandlePendingError(now_ms);

    switch (s_state)
    {
        case LINE_UART_STATE_POWER_WAIT:
            if (LineUart_Elapsed(now_ms,
                                 s_init_timestamp_ms,
                                 BSP_LINE_UART_POWER_STABLE_MS))
            {
                LineUart_TrySendMode(now_ms);
            }
            break;

        case LINE_UART_STATE_MODE_TX_WAIT:
            if (s_tx_complete)
            {
                s_tx_complete = false;
                s_mode_command_sent = true;
                s_state = LINE_UART_STATE_MODE_SETTLE;
                s_state_timestamp_ms = now_ms;
                s_status = BSP_LINE_UART_STATUS_MODE_SETTLE;
            }
            break;

        case LINE_UART_STATE_MODE_SETTLE:
            if (LineUart_Elapsed(now_ms,
                                 s_state_timestamp_ms,
                                 BSP_LINE_UART_MODE_SETTLE_MS))
            {
                LineUart_EnterReady(now_ms);
            }
            break;

        case LINE_UART_STATE_READY:
            if (LineUart_TimeReached(now_ms, s_next_query_ms))
            {
                LineUart_TryStartQuery(now_ms);
            }
            break;

        case LINE_UART_STATE_RESPONSE_WAIT:
            if (s_rx_complete)
            {
                uint32_t primask = LineUart_EnterCritical();

                s_rx_complete = false;
                s_latest_raw_state = s_rx_byte;
                s_sequence++;
                s_response_count++;
                s_latest_timestamp_ms = now_ms;
                s_status = BSP_LINE_UART_STATUS_OK;
                s_state = LINE_UART_STATE_READY;
                s_next_query_ms =
                    s_query_timestamp_ms + BSP_LINE_UART_QUERY_PERIOD_MS;

                LineUart_ExitCritical(primask);
            }
            else if (LineUart_Elapsed(now_ms,
                                      s_query_timestamp_ms,
                                      BSP_LINE_UART_RESPONSE_TIMEOUT_MS))
            {
                (void)HAL_UART_AbortReceive(&huart4);
                s_timeout_count++;
                s_state = LINE_UART_STATE_READY;
                s_next_query_ms =
                    s_query_timestamp_ms + BSP_LINE_UART_QUERY_PERIOD_MS;
                s_status = BSP_LINE_UART_STATUS_TIMEOUT;
            }
            break;

        case LINE_UART_STATE_STOPPED:
        default:
            break;
    }
}

void BSP_LineUart_Stop(void)
{
    uint32_t primask;

    if (!s_initialized)
    {
        return;
    }

    (void)HAL_UART_Abort(&huart4);

    primask = LineUart_EnterCritical();
    s_initialized = false;
    s_mode_command_sent = false;
    s_tx_complete = false;
    s_rx_complete = false;
    s_uart_error_pending = false;
    s_state = LINE_UART_STATE_STOPPED;
    s_status = BSP_LINE_UART_STATUS_UNINITIALIZED;
    LineUart_ExitCritical(primask);
}

bool BSP_LineUart_IsInitialized(void)
{
    return s_initialized;
}

bool BSP_LineUart_IsReady(void)
{
    if (!s_initialized || !s_mode_command_sent)
    {
        return false;
    }

    return ((s_state == LINE_UART_STATE_READY) ||
            (s_state == LINE_UART_STATE_RESPONSE_WAIT));
}

bool BSP_LineUart_IsDataValid(void)
{
    uint32_t now_ms;
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint32_t primask;

    if (!s_initialized)
    {
        return false;
    }

    primask = LineUart_EnterCritical();
    timestamp_ms = s_latest_timestamp_ms;
    sequence = s_sequence;
    LineUart_ExitCritical(primask);

    if (sequence == 0U)
    {
        return false;
    }

    now_ms = HAL_GetTick();
    return ((uint32_t)(now_ms - timestamp_ms) <=
            BSP_LINE_UART_DATA_VALID_MS);
}

bool BSP_LineUart_GetSnapshot(BspLineUartSnapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
    {
        return false;
    }

    primask = LineUart_EnterCritical();

    snapshot->raw_state = s_latest_raw_state;
    snapshot->sequence = s_sequence;
    snapshot->timestamp_ms = s_latest_timestamp_ms;
    snapshot->query_count = s_query_count;
    snapshot->response_count = s_response_count;
    snapshot->timeout_count = s_timeout_count;
    snapshot->uart_error_count = s_uart_error_count;
    snapshot->hal_busy_count = s_hal_busy_count;
    snapshot->last_uart_error = s_last_uart_error;
    snapshot->status = s_status;
    snapshot->initialized = s_initialized;
    snapshot->mode_command_sent = s_mode_command_sent;

    LineUart_ExitCritical(primask);

    snapshot->data_valid = BSP_LineUart_IsDataValid();
    return true;
}

const char *BSP_LineUart_StatusName(BspLineUartStatus_t status)
{
    switch (status)
    {
        case BSP_LINE_UART_STATUS_UNINITIALIZED:
            return "UNINIT";
        case BSP_LINE_UART_STATUS_POWER_WAIT:
            return "POWER_WAIT";
        case BSP_LINE_UART_STATUS_MODE_SENDING:
            return "MODE_TX";
        case BSP_LINE_UART_STATUS_MODE_SETTLE:
            return "MODE_WAIT";
        case BSP_LINE_UART_STATUS_READY:
            return "READY";
        case BSP_LINE_UART_STATUS_WAIT_RESPONSE:
            return "RX_WAIT";
        case BSP_LINE_UART_STATUS_OK:
            return "OK";
        case BSP_LINE_UART_STATUS_TIMEOUT:
            return "TIMEOUT";
        case BSP_LINE_UART_STATUS_UART_ERROR:
            return "UART_ERR";
        case BSP_LINE_UART_STATUS_HAL_BUSY:
            return "HAL_BUSY";
        default:
            return "UNKNOWN";
    }
}

void BSP_LineUart_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    s_tx_complete = true;
}

void BSP_LineUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    if (s_state == LINE_UART_STATE_RESPONSE_WAIT)
    {
        s_rx_complete = true;
    }
}

void BSP_LineUart_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    s_last_uart_error = huart->ErrorCode;
    s_uart_error_count++;
    s_status = BSP_LINE_UART_STATUS_UART_ERROR;
    s_uart_error_pending = true;
}
