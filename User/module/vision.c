#include "vision.h"

#include "bsp_vision_uart.h"
#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <string.h>

static VisionProtocolParser_t s_parser;
static VisionBallFrame_t s_latest_frame;
static uint32_t s_sequence = 0U;
static uint32_t s_timestamp_ms = 0U;
static bool s_initialized = false;
static bool s_has_frame = false;

bool Vision_Init(void)
{
    s_initialized = false;
    s_has_frame = false;
    s_sequence = 0U;
    s_timestamp_ms = 0U;
    memset(&s_latest_frame, 0, sizeof(s_latest_frame));
    VisionProtocol_Init(&s_parser);

    if (!BSP_VisionUart_Init())
    {
        return false;
    }

    s_initialized = true;
    return true;
}

void Vision_Update(void)
{
    VisionBallFrame_t frame;
    uint8_t byte;

    if (!s_initialized)
    {
        return;
    }

    BSP_VisionUart_Process();
    while (BSP_VisionUart_PopByte(&byte))
    {
        if (VisionProtocol_FeedByte(&s_parser, byte, &frame))
        {
            s_latest_frame = frame;
            s_sequence++;
            s_timestamp_ms = HAL_GetTick();
            s_has_frame = true;
        }
    }
}

void Vision_Stop(void)
{
    BSP_VisionUart_Stop();
    s_initialized = false;
    s_has_frame = false;
}

bool Vision_GetStatus(VisionStatus_t *status)
{
    BspVisionUartStatus_t uart_status;
    uint32_t now_ms;

    if ((status == NULL) || !BSP_VisionUart_GetStatus(&uart_status))
    {
        return false;
    }

    now_ms = HAL_GetTick();
    status->frame = s_latest_frame;
    status->sequence = s_sequence;
    status->timestamp_ms = s_timestamp_ms;
    status->valid_frame_count = s_parser.valid_frame_count;
    status->invalid_frame_count = s_parser.invalid_frame_count;
    status->protocol_overflow_count = s_parser.overflow_count;
    status->received_byte_count = uart_status.received_byte_count;
    status->uart_overflow_count = uart_status.overflow_count;
    status->uart_error_count = uart_status.uart_error_count;
    status->uart_restart_count = uart_status.restart_count;
    status->last_uart_error = uart_status.last_uart_error;
    status->queued_byte_count = uart_status.queued_byte_count;
    status->initialized = s_initialized;
    status->has_frame = s_has_frame;
    status->data_valid = s_has_frame &&
        ((uint32_t)(now_ms - s_timestamp_ms) <= VISION_DATA_VALID_MS);
    return true;
}
