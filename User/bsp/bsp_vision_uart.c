#include "bsp_vision_uart.h"

#include "usart.h"

#include <stddef.h>

static uint8_t s_ring[BSP_VISION_UART_RING_CAPACITY];
static uint8_t s_rx_byte = 0U;

static volatile uint16_t s_head = 0U;
static volatile uint16_t s_tail = 0U;
static volatile uint32_t s_received_byte_count = 0U;
static volatile uint32_t s_overflow_count = 0U;
static volatile uint32_t s_uart_error_count = 0U;
static volatile uint32_t s_restart_count = 0U;
static volatile uint32_t s_last_uart_error = 0U;
static volatile bool s_initialized = false;
static volatile bool s_receiving = false;
static volatile bool s_restart_pending = false;

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

static uint16_t VisionUart_NextIndex(uint16_t index)
{
    return (uint16_t)((index + 1U) % BSP_VISION_UART_RING_CAPACITY);
}

static bool VisionUart_StartReceive(void)
{
    HAL_StatusTypeDef result;

    result = HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U);
    s_receiving = result == HAL_OK;
    if (!s_receiving)
    {
        s_restart_count++;
        s_restart_pending = true;
    }
    return s_receiving;
}

bool BSP_VisionUart_Init(void)
{
    if (huart6.Instance != USART6)
    {
        return false;
    }

    (void)HAL_UART_Abort(&huart6);
    s_head = 0U;
    s_tail = 0U;
    s_received_byte_count = 0U;
    s_overflow_count = 0U;
    s_uart_error_count = 0U;
    s_restart_count = 0U;
    s_last_uart_error = 0U;
    s_receiving = false;
    s_restart_pending = false;
    s_initialized = true;

    if (!VisionUart_StartReceive())
    {
        s_initialized = false;
        return false;
    }
    return true;
}

void BSP_VisionUart_Process(void)
{
    if (!s_initialized || !s_restart_pending)
    {
        return;
    }

    s_restart_pending = false;
    (void)HAL_UART_AbortReceive(&huart6);
    (void)VisionUart_StartReceive();
}

void BSP_VisionUart_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    s_initialized = false;
    s_receiving = false;
    s_restart_pending = false;
    (void)HAL_UART_AbortReceive(&huart6);
}

bool BSP_VisionUart_PopByte(uint8_t *byte)
{
    uint16_t tail;

    if ((byte == NULL) || !s_initialized || (s_tail == s_head))
    {
        return false;
    }

    tail = s_tail;
    *byte = s_ring[tail];
    s_tail = VisionUart_NextIndex(tail);
    return true;
}

bool BSP_VisionUart_GetStatus(BspVisionUartStatus_t *status)
{
    uint16_t head;
    uint16_t tail;
    uint32_t primask;

    if (status == NULL)
    {
        return false;
    }

    primask = VisionUart_EnterCritical();
    head = s_head;
    tail = s_tail;
    status->received_byte_count = s_received_byte_count;
    status->overflow_count = s_overflow_count;
    status->uart_error_count = s_uart_error_count;
    status->restart_count = s_restart_count;
    status->last_uart_error = s_last_uart_error;
    status->initialized = s_initialized;
    status->receiving = s_receiving;
    VisionUart_ExitCritical(primask);

    status->queued_byte_count = (head >= tail)
        ? (uint16_t)(head - tail)
        : (uint16_t)(BSP_VISION_UART_RING_CAPACITY - tail + head);
    return true;
}

void BSP_VisionUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (!s_initialized || (huart == NULL) || (huart->Instance != USART6))
    {
        return;
    }

    next = VisionUart_NextIndex(s_head);
    if (next == s_tail)
    {
        s_overflow_count++;
    }
    else
    {
        s_ring[s_head] = s_rx_byte;
        s_head = next;
        s_received_byte_count++;
    }

    s_receiving = false;
    (void)VisionUart_StartReceive();
}

void BSP_VisionUart_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (!s_initialized || (huart == NULL) || (huart->Instance != USART6))
    {
        return;
    }

    s_last_uart_error = huart->ErrorCode;
    s_uart_error_count++;
    s_receiving = false;
    s_restart_pending = true;
}
