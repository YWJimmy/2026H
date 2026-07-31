#include "test_vision_uart.h"

#include "bsp_debug_uart.h"
#include "stm32f4xx_hal.h"
#include "vision.h"

#define TEST_VISION_UART_REPORT_PERIOD_MS    100U

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;

bool Test_VisionUart_Init(void)
{
    s_initialized = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!Vision_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,VISION_UART_INIT,CHECK=USART6_PG9_PG14_115200_8N1_IRQ\r\n");
        return false;
    }

    s_last_report_ms = HAL_GetTick();
    s_initialized = true;
    (void)BSP_Debug_Printf(
        "TEST,VISION_UART,START,RX=PG9,TX=PG14,BAUD=115200,"
        "FRAME=SB_FOUND_X1_Y1_X2_Y2_CX_CY_SCORE,"
        "PHYSICAL_X_UNIT=MM\r\n");
    return true;
}

void Test_VisionUart_Update(void)
{
    VisionStatus_t status;
    uint32_t now_ms;
    uint32_t age_ms;

    BSP_DebugUart_Process();
    if (!s_initialized)
    {
        return;
    }

    Vision_Update();
    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_report_ms) <
        TEST_VISION_UART_REPORT_PERIOD_MS)
    {
        return;
    }
    s_last_report_ms = now_ms;

    if (!Vision_GetStatus(&status))
    {
        return;
    }

    age_ms = status.has_frame
        ? (uint32_t)(now_ms - status.timestamp_ms)
        : 0U;
    (void)BSP_Debug_Printf(
        "VU,SEQ=%lu,FOUND=%u,CX=%u,CY=%u,X_MM=%d,SCORE=%u,VALID=%u,AGE=%lu,"
        "RX=%lu,Q=%u,VF=%lu,IF=%lu,PO=%lu,RO=%lu,UE=%lu,RR=%lu,LE=0x%08lX\r\n",
        (unsigned long)status.sequence,
        status.frame.found ? 1U : 0U,
        (unsigned int)status.frame.center_x,
        (unsigned int)status.frame.center_y,
        (int)status.frame.physical_x_mm,
        (unsigned int)status.frame.score_milli,
        status.data_valid ? 1U : 0U,
        (unsigned long)age_ms,
        (unsigned long)status.received_byte_count,
        (unsigned int)status.queued_byte_count,
        (unsigned long)status.valid_frame_count,
        (unsigned long)status.invalid_frame_count,
        (unsigned long)status.protocol_overflow_count,
        (unsigned long)status.uart_overflow_count,
        (unsigned long)status.uart_error_count,
        (unsigned long)status.uart_restart_count,
        (unsigned long)status.last_uart_error);
}

void Test_VisionUart_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    Vision_Stop();
    s_initialized = false;
    (void)BSP_Debug_Printf("TEST,VISION_UART,STOP\r\n");
}
