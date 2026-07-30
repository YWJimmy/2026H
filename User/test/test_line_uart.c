#include "test_line_uart.h"

#include "bsp_debug_uart.h"
#include "bsp_line_uart.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_LINE_UART_REPORT_PERIOD_MS    50U

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;

static uint8_t Test_LineUart_GetBit(uint8_t value, uint8_t bit)
{
    return (uint8_t)((value >> bit) & 0x01U);
}

bool Test_LineUart_Init(void)
{
    s_initialized = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_LineUart_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,LINE_UART_INIT,CHECK=UART4_PC10_PC11_115200_8N1_IRQ\r\n");
        return false;
    }

    s_last_report_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,LINE_UART,START,PORT=UART4,TX=PC10,RX=PC11,"
        "BAUD=115200,MODE=0x00,QUERY=0x01,PERIOD_MS=%u\r\n",
        (unsigned int)BSP_LINE_UART_QUERY_PERIOD_MS);
    (void)BSP_Debug_Printf(
        "TEST,LINE_UART,NOTICE=RAW_ONLY,POWER_CYCLE_SENSOR_WITH_MCU,"
        "POLARITY_AND_DIRECTION_UNCONFIRMED\r\n");

    return true;
}

void Test_LineUart_Update(void)
{
    BspLineUartSnapshot_t snapshot;
    uint32_t now_ms;
    uint32_t age_ms;

    BSP_LineUart_Process();
    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_report_ms) <
        TEST_LINE_UART_REPORT_PERIOD_MS)
    {
        return;
    }
    s_last_report_ms += TEST_LINE_UART_REPORT_PERIOD_MS;

    if (!BSP_LineUart_GetSnapshot(&snapshot))
    {
        return;
    }

    age_ms = (snapshot.sequence == 0U)
        ? 0U
        : (uint32_t)(now_ms - snapshot.timestamp_ms);

    (void)BSP_Debug_Printf(
        "LU,SEQ=%lu,RAW=0x%02X,BITS=%u%u%u%u%u%u%u%u,"
        "S=%u,%u,%u,%u,%u,%u,%u,%u,"
        "VALID=%u,AGE=%lu,Q=%lu,R=%lu,TO=%lu,UE=%lu,HB=%lu,ST=%s\r\n",
        (unsigned long)snapshot.sequence,
        (unsigned int)snapshot.raw_state,
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 7U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 6U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 5U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 4U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 3U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 2U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 1U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 0U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 0U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 1U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 2U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 3U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 4U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 5U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 6U),
        (unsigned int)Test_LineUart_GetBit(snapshot.raw_state, 7U),
        snapshot.data_valid ? 1U : 0U,
        (unsigned long)age_ms,
        (unsigned long)snapshot.query_count,
        (unsigned long)snapshot.response_count,
        (unsigned long)snapshot.timeout_count,
        (unsigned long)snapshot.uart_error_count,
        (unsigned long)snapshot.hal_busy_count,
        BSP_LineUart_StatusName(snapshot.status));
}

void Test_LineUart_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    BSP_LineUart_Stop();
    s_initialized = false;
    (void)BSP_Debug_Printf("TEST,LINE_UART,STOP\r\n");
}

bool Test_LineUart_IsInitialized(void)
{
    return s_initialized;
}
