#include "bsp_vision_uart.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>

/* K230D: "SB,flag,x1,y1,x2,y2,cx,cy,score\r\n"  (max ~40 chars) */
#define VISION_UART_LINE_BUFFER_SIZE    64U

/* Restart RX if no byte received within this window (ms). */
#define VISION_UART_RX_TIMEOUT_MS       500U

static bool s_initialized = false;

/* 中断逐字节接收：HAL_UART_Receive_IT 用 */
static volatile uint8_t s_rx_byte = 0U;

/* 行缓冲（仅 ISR 写入，主循环不访问） */
static char s_line_buffer[VISION_UART_LINE_BUFFER_SIZE];
static volatile uint8_t s_line_index = 0U;

/* 解析结果（ISR 写入，主循环读取 —— 需关中断保护） */
static BspVisionDetection_t s_detection;
static volatile bool s_new_detection = false;

/* 统计 */
static volatile uint32_t s_frame_count = 0U;
static volatile uint32_t s_error_count = 0U;

/* 最后收到字节的时间（ISR 和主循环都可能读，但只用 HAL_GetTick 短暂窗口）*/
static volatile uint32_t s_last_byte_ms = 0U;

/* ---- 临界区辅助 ---- */

static uint32_t VisionUart_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void VisionUart_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/* ---- 内部函数 ---- */

static void VisionUart_StartReceive(void)
{
    /*
     * 在 ISR 或 Init 中调用。
     * 忽略返回值：如果 USART6 正忙，说明硬件异常，但不应在此阻塞。
     */
    (void)HAL_UART_Receive_IT(&huart6, (uint8_t *)&s_rx_byte, 1U);
}

static void VisionUart_ParseLine(const char *line, uint8_t length)
{
    int has_target = 0;
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    int cx = 0, cy = 0, score_milli = 0;
    int matched;
    uint32_t primask;

    /*
     * 最短有效帧: "SB,0,0,0,0,0,0,0,0" = 20 字符。
     * 行尾可能带 \r，因此 length 至少为 20。
     */
    if ((length < 20U) || (line == NULL))
    {
        s_error_count++;
        return;
    }

    /* 校验帧头 */
    if ((line[0] != 'S') || (line[1] != 'B') || (line[2] != ','))
    {
        s_error_count++;
        return;
    }

    matched = sscanf(line,
                     "SB,%d,%d,%d,%d,%d,%d,%d,%d",
                     &has_target,
                     &x1, &y1, &x2, &y2,
                     &cx, &cy,
                     &score_milli);

    if (matched != 8)
    {
        s_error_count++;
        return;
    }

    if ((has_target != 0) && (has_target != 1))
    {
        s_error_count++;
        return;
    }

    /* 写入解析结果 */
    primask = VisionUart_EnterCritical();

    s_detection.has_target  = (has_target != 0);
    s_detection.x1          = (uint16_t)x1;
    s_detection.y1          = (uint16_t)y1;
    s_detection.x2          = (uint16_t)x2;
    s_detection.y2          = (uint16_t)y2;
    s_detection.cx          = (uint16_t)cx;
    s_detection.cy          = (uint16_t)cy;
    s_detection.score_milli = (uint16_t)score_milli;
    s_detection.timestamp_ms = HAL_GetTick();

    s_new_detection = true;
    s_frame_count++;

    VisionUart_ExitCritical(primask);
}

/* ---- 公开接口 ---- */

bool BSP_VisionUart_Init(void)
{
    uint32_t primask;

    primask = VisionUart_EnterCritical();

    s_initialized = false;
    s_line_index = 0U;
    s_new_detection = false;
    s_frame_count = 0U;
    s_error_count = 0U;
    s_last_byte_ms = 0U;

    memset((void *)s_line_buffer, 0, sizeof(s_line_buffer));
    memset((void *)&s_detection, 0, sizeof(s_detection));

    VisionUart_ExitCritical(primask);

    /* 校验 USART6 已由 CubeMX 配置 */
    if (huart6.Instance != USART6)
    {
        return false;
    }

    s_initialized = true;
    s_last_byte_ms = HAL_GetTick();

    /* 清空可能残留的接收缓冲 */
    (void)HAL_UART_AbortReceive(&huart6);

    VisionUart_StartReceive();
    return true;
}

bool BSP_VisionUart_IsInitialized(void)
{
    return s_initialized;
}

void BSP_VisionUart_Process(void)
{
    uint32_t now_ms;

    if (!s_initialized)
    {
        return;
    }

    /*
     * 超时检测：如果超过 VISION_UART_RX_TIMEOUT_MS
     * 没有收到新字节，尝试中止当前接收并重新启动。
     * 这可以恢复因噪声或中断丢失导致的接收停滞。
     */
    now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - s_last_byte_ms) >=
        VISION_UART_RX_TIMEOUT_MS)
    {
        s_line_index = 0U;
        (void)HAL_UART_AbortReceive(&huart6);
        VisionUart_StartReceive();
        s_last_byte_ms = HAL_GetTick();
    }
}

bool BSP_VisionUart_HasNewDetection(void)
{
    return s_new_detection;
}

BspVisionDetection_t BSP_VisionUart_GetDetection(void)
{
    BspVisionDetection_t result;
    uint32_t primask;

    primask = VisionUart_EnterCritical();

    result = s_detection;
    s_new_detection = false;

    VisionUart_ExitCritical(primask);

    return result;
}

void BSP_VisionUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
    char ch;

    if ((!s_initialized) || (huart != &huart6))
    {
        return;
    }

    s_last_byte_ms = HAL_GetTick();
    ch = (char)s_rx_byte;

    /*
     * 跳过 \r。收到 \n 时触发行解析。
     * 如果 K230D 异常只发送 \r（不带 \n），也将 \r 视为行尾。
     */
    if (ch == '\r')
    {
        /*
         * 先将当前行（如果有）解析，再准备接收下一行。
         * 多数情况下 \r 后紧跟 \n，所以选在 \n 时解析。
         * 这里只跳过 \r，不做其他动作。
         */
        VisionUart_StartReceive();
        return;
    }

    if (ch == '\n')
    {
        if ((s_line_index > 0U) &&
            (s_line_index < VISION_UART_LINE_BUFFER_SIZE))
        {
            s_line_buffer[s_line_index] = '\0';
            VisionUart_ParseLine(s_line_buffer, s_line_index);
        }

        s_line_index = 0U;
        VisionUart_StartReceive();
        return;
    }

    /* 普通字符：追加到行缓冲 */
    if (s_line_index < (VISION_UART_LINE_BUFFER_SIZE - 1U))
    {
        s_line_buffer[s_line_index] = ch;
        s_line_index++;
    }
    else
    {
        /*
         * 行过长，丢弃整行。
         * 不清 s_error_count —— 这不代表 K230D 协议错误，
         * 更可能是缓冲区配小了或串口噪声。
         */
        s_line_index = 0U;
    }

    VisionUart_StartReceive();
}

void BSP_VisionUart_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((!s_initialized) || (huart != &huart6))
    {
        return;
    }

    /*
     * USART6 发生错误（噪声、帧错误、溢出等）。
     * 丢弃当前行，中止并重启接收。
     */
    s_line_index = 0U;
    s_error_count++;
    s_last_byte_ms = HAL_GetTick();

    (void)HAL_UART_AbortReceive(&huart6);
    VisionUart_StartReceive();
}

uint32_t BSP_VisionUart_GetFrameCount(void)
{
    return s_frame_count;
}

uint32_t BSP_VisionUart_GetErrorCount(void)
{
    return s_error_count;
}

/* ---- Stubs for vision.c compatibility ---- */
void BSP_VisionUart_Stop(void)
{
    s_initialized = false;
    (void)HAL_UART_AbortReceive(&huart6);
}

bool BSP_VisionUart_PopByte(uint8_t *byte)
{
    (void)byte;
    return false;
}

bool BSP_VisionUart_GetStatus(BspVisionUartStatus_t *status)
{
    if (status == NULL) return false;
    status->received_byte_count = s_frame_count;
    status->overflow_count = 0U;
    status->uart_error_count = s_error_count;
    status->restart_count = 0U;
    status->last_uart_error = 0U;
    status->queued_byte_count = 0U;
    return true;
}
