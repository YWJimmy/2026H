#include "bsp_debug_uart.h"

#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef BSP_DEBUG_UART_QUEUE_DEPTH
#define BSP_DEBUG_UART_QUEUE_DEPTH          8U
#endif

#ifndef BSP_DEBUG_UART_MESSAGE_MAX_LENGTH
#define BSP_DEBUG_UART_MESSAGE_MAX_LENGTH   160U
#endif

#if (BSP_DEBUG_UART_QUEUE_DEPTH < 2U)
#error "BSP_DEBUG_UART_QUEUE_DEPTH must be at least 2"
#endif

#if (BSP_DEBUG_UART_MESSAGE_MAX_LENGTH < 16U)
#error "BSP_DEBUG_UART_MESSAGE_MAX_LENGTH must be at least 16"
#endif

typedef struct
{
    uint8_t data[BSP_DEBUG_UART_MESSAGE_MAX_LENGTH];
    uint16_t length;
} DebugUartMessage_t;

static DebugUartMessage_t s_queue[BSP_DEBUG_UART_QUEUE_DEPTH];
static volatile uint8_t s_head = 0U;
static volatile uint8_t s_tail = 0U;
static volatile uint8_t s_count = 0U;
static volatile bool s_dma_busy = false;
static volatile bool s_initialized = false;
static volatile uint32_t s_dropped_count = 0U;

static uint32_t DebugUart_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void DebugUart_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t DebugUart_NextIndex(uint8_t index)
{
    index++;
    if (index >= BSP_DEBUG_UART_QUEUE_DEPTH)
    {
        index = 0U;
    }
    return index;
}

bool BSP_DebugUart_Init(void)
{
    uint32_t primask;

    primask = DebugUart_EnterCritical();
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_dma_busy = false;
    s_dropped_count = 0U;
    s_initialized = false;
    DebugUart_ExitCritical(primask);

    if ((huart1.Instance != USART1) || (huart1.hdmatx == NULL))
    {
        return false;
    }

    primask = DebugUart_EnterCritical();
    s_initialized = true;
    DebugUart_ExitCritical(primask);

    return true;
}

bool BSP_DebugUart_IsInitialized(void)
{
    return s_initialized;
}

bool BSP_DebugUart_Write(const uint8_t *data, size_t length)
{
    uint32_t primask;
    uint16_t stored_length;
    uint8_t write_index;

    if ((!s_initialized) || (data == NULL) || (length == 0U))
    {
        return false;
    }

    if (length > BSP_DEBUG_UART_MESSAGE_MAX_LENGTH)
    {
        stored_length = (uint16_t)BSP_DEBUG_UART_MESSAGE_MAX_LENGTH;
    }
    else
    {
        stored_length = (uint16_t)length;
    }

    primask = DebugUart_EnterCritical();

    if (s_count >= BSP_DEBUG_UART_QUEUE_DEPTH)
    {
        s_dropped_count++;
        DebugUart_ExitCritical(primask);
        return false;
    }

    write_index = s_head;
    memcpy(s_queue[write_index].data, data, stored_length);
    s_queue[write_index].length = stored_length;
    s_head = DebugUart_NextIndex(s_head);
    s_count++;

    DebugUart_ExitCritical(primask);

    BSP_DebugUart_Process();
    return true;
}

int BSP_Debug_Printf(const char *format, ...)
{
    char buffer[BSP_DEBUG_UART_MESSAGE_MAX_LENGTH];
    va_list args;
    int formatted_length;
    size_t queued_length;

    if ((!s_initialized) || (format == NULL))
    {
        return -1;
    }

    va_start(args, format);
    formatted_length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (formatted_length < 0)
    {
        return -1;
    }

    if ((size_t)formatted_length >= sizeof(buffer))
    {
        queued_length = sizeof(buffer) - 1U;
    }
    else
    {
        queued_length = (size_t)formatted_length;
    }

    if (!BSP_DebugUart_Write((const uint8_t *)buffer, queued_length))
    {
        return -2;
    }

    return (int)queued_length;
}

void BSP_DebugUart_Process(void)
{
    uint32_t primask;
    uint8_t *data = NULL;
    uint16_t length = 0U;
    HAL_StatusTypeDef status;

    if (!s_initialized)
    {
        return;
    }

    primask = DebugUart_EnterCritical();

    if ((!s_dma_busy) && (s_count > 0U))
    {
        s_dma_busy = true;
        data = s_queue[s_tail].data;
        length = s_queue[s_tail].length;
    }

    DebugUart_ExitCritical(primask);

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    status = HAL_UART_Transmit_DMA(&huart1, data, length);
    if (status != HAL_OK)
    {
        primask = DebugUart_EnterCritical();
        s_dma_busy = false;
        DebugUart_ExitCritical(primask);
    }
}

void BSP_DebugUart_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if ((!s_initialized) || (huart != &huart1))
    {
        return;
    }

    primask = DebugUart_EnterCritical();

    if (s_dma_busy && (s_count > 0U))
    {
        s_tail = DebugUart_NextIndex(s_tail);
        s_count--;
    }

    s_dma_busy = false;
    DebugUart_ExitCritical(primask);

    BSP_DebugUart_Process();
}

void BSP_DebugUart_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if ((!s_initialized) || (huart != &huart1))
    {
        return;
    }

    (void)HAL_UART_AbortTransmit(huart);

    primask = DebugUart_EnterCritical();

    if (s_dma_busy && (s_count > 0U))
    {
        s_tail = DebugUart_NextIndex(s_tail);
        s_count--;
        s_dropped_count++;
    }

    s_dma_busy = false;
    DebugUart_ExitCritical(primask);

    BSP_DebugUart_Process();
}

bool BSP_DebugUart_IsBusy(void)
{
    return s_dma_busy;
}

uint32_t BSP_DebugUart_GetDroppedCount(void)
{
    return s_dropped_count;
}

void BSP_DebugUart_ClearPending(void)
{
    uint32_t primask;

    primask = DebugUart_EnterCritical();

    if (s_dma_busy && (s_count > 0U))
    {
        s_count = 1U;
        s_head = DebugUart_NextIndex(s_tail);
    }
    else
    {
        s_count = 0U;
        s_head = s_tail;
    }

    DebugUart_ExitCritical(primask);
}
