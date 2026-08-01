#include "test_line_adc.h"

#include "bsp_debug_uart.h"
#include "bsp_line_adc.h"
#include "stm32f4xx_hal.h"

#define TEST_LINE_ADC_REPORT_PERIOD_MS     100U

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;

bool Test_LineAdc_Init(void)
{
    s_initialized = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_LineAdc_Init())
    {
        (void)BSP_Debug_Printf("ERR,LINE_ADC_INIT\r\n");
        return false;
    }

    if (!BSP_LineAdc_Start())
    {
        (void)BSP_Debug_Printf("ERR,LINE_ADC_START\r\n");
        return false;
    }

    s_last_report_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,LINE_ADC,START,CH=8,RATE_HZ=1000,STABLE_MS=%u\r\n",
        (unsigned int)BSP_LINE_ADC_STABILIZE_MS);

    return true;
}

void Test_LineAdc_Update(void)
{
    BspLineAdcSnapshot_t snapshot;
    uint32_t now_ms;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_report_ms) <
        TEST_LINE_ADC_REPORT_PERIOD_MS)
    {
        return;
    }
    s_last_report_ms = now_ms;

    if (!BSP_LineAdc_GetSnapshot(&snapshot))
    {
        (void)BSP_Debug_Printf(
            "ADC8,VALID=0,SEQ=%lu,ERR=%lu\r\n",
            (unsigned long)BSP_LineAdc_GetSequence(),
            (unsigned long)BSP_LineAdc_GetErrorCount());
        return;
    }

    (void)BSP_Debug_Printf(
        "ADC8,SEQ=%lu,ERR=%lu,RAW=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned long)snapshot.sequence,
        (unsigned long)BSP_LineAdc_GetErrorCount(),
        (unsigned int)snapshot.raw[0],
        (unsigned int)snapshot.raw[1],
        (unsigned int)snapshot.raw[2],
        (unsigned int)snapshot.raw[3],
        (unsigned int)snapshot.raw[4],
        (unsigned int)snapshot.raw[5],
        (unsigned int)snapshot.raw[6],
        (unsigned int)snapshot.raw[7]);
}

void Test_LineAdc_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    (void)BSP_LineAdc_Stop();
    s_initialized = false;
    (void)BSP_Debug_Printf("TEST,LINE_ADC,STOP\r\n");
}

bool Test_LineAdc_IsInitialized(void)
{
    return s_initialized;
}
