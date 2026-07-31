#include "test_oled_fullscreen_timer.h"

#include "bsp_debug_uart.h"
#include "bsp_oled.h"
#include "oled_fullscreen_timer.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_TIMER_CYCLE_MS          ((uint32_t)100000U)
#define TEST_TIMER_REPORT_MS         ((uint32_t)1000U)

static bool s_initialized = false;
static uint32_t s_start_ms = 0U;
static uint32_t s_last_report_ms = 0U;

bool Test_OledFullscreenTimer_Init(void)
{
    s_initialized = false;
    if (!BSP_DebugUart_Init() || !BSP_Oled_Init())
    {
        return false;
    }

    s_start_ms = HAL_GetTick();
    s_last_report_ms = s_start_ms;
    OledFullscreenTimer_Reset();
    OledFullscreenTimer_ShowElapsedMs(0U);
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,OLED_FULLSCREEN_TIMER,START,FORMAT=XX.X,RANGE=00.0-99.9\r\n");
    return true;
}

void Test_OledFullscreenTimer_Update(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
    uint32_t deciseconds;

    BSP_DebugUart_Process();
    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();
    elapsed_ms =
        (uint32_t)(now_ms - s_start_ms) %
        TEST_TIMER_CYCLE_MS;
    OledFullscreenTimer_ShowElapsedMs(elapsed_ms);
    BSP_Oled_Process();

    if ((uint32_t)(now_ms - s_last_report_ms) >=
        TEST_TIMER_REPORT_MS)
    {
        s_last_report_ms = now_ms;
        deciseconds = elapsed_ms / 100U;
        (void)BSP_Debug_Printf(
            "OLED_TIMER,DISPLAY=%02lu.%lu,ONLINE=%u\r\n",
            (unsigned long)(deciseconds / 10U),
            (unsigned long)(deciseconds % 10U),
            BSP_Oled_IsOnline() ? 1U : 0U);
    }
}

void Test_OledFullscreenTimer_Stop(void)
{
    s_initialized = false;
    BSP_Oled_Clear();
    (void)BSP_Debug_Printf(
        "TEST,OLED_FULLSCREEN_TIMER,STOP\r\n");
}
