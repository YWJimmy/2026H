#include "test_encoder.h"

#include "bsp_debug_uart.h"
#include "bsp_encoder.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_ENCODER_REPORT_PERIOD_MS    100U

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;

bool Test_Encoder_Init(void)
{
    s_initialized = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_Encoder_Init())
    {
        (void)BSP_Debug_Printf("ERR,ENCODER_INIT\r\n");
        return false;
    }

    BSP_Encoder_Reset();
    s_last_report_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,ENCODER,START,CPR=%ld,PERIOD_MS=%lu\r\n",
        (long)BSP_ENCODER_COUNTS_PER_REV,
        (unsigned long)TEST_ENCODER_REPORT_PERIOD_MS);

    return true;
}

void Test_Encoder_Update(void)
{
    uint32_t now_ms;
    int16_t left_delta;
    int16_t right_delta;
    int32_t left_total;
    int32_t right_total;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_report_ms) < TEST_ENCODER_REPORT_PERIOD_MS)
    {
        return;
    }

    s_last_report_ms = now_ms;

    left_delta = BSP_Encoder_GetLeftDelta();
    right_delta = BSP_Encoder_GetRightDelta();
    left_total = BSP_Encoder_GetLeftTotal();
    right_total = BSP_Encoder_GetRightTotal();

    (void)BSP_Debug_Printf(
        "ENC,LD=%d,LT=%ld,RD=%d,RT=%ld\r\n",
        (int)left_delta,
        (long)left_total,
        (int)right_delta,
        (long)right_total);
}

void Test_Encoder_Reset(void)
{
    if (!s_initialized)
    {
        return;
    }

    BSP_Encoder_Reset();
    s_last_report_ms = HAL_GetTick();
    (void)BSP_Debug_Printf("TEST,ENCODER,RESET\r\n");
}

bool Test_Encoder_IsInitialized(void)
{
    return s_initialized;
}
