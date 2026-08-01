#include "test_line_follow.h"

#include "bsp_debug_uart.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "stm32f4xx_hal.h"

#define TEST_LINE_FOLLOW_REPORT_PERIOD_MS      100U

static bool s_initialized = false;
static bool s_has_result = false;
static uint32_t s_last_report_ms = 0U;
static LineFollowResult_t s_latest_result;

bool Test_LineFollow_Init(void)
{
    s_initialized = false;
    s_has_result = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!LineSensor_Init())
    {
        (void)BSP_Debug_Printf("ERR,LINE_SENSOR_INIT\r\n");
        return false;
    }

    LineFollow_Init();
    s_last_report_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,LINE_FOLLOW,START,BACKEND=%s,CONFIG_OK=%u,"
        "POS_RANGE=-7000_TO_7000\r\n",
        LineSensor_GetBackendName(),
        LineSensor_IsBackendConfigConfirmed() ? 1U : 0U);

    return true;
}

void Test_LineFollow_Update(void)
{
    LineSensorFrame_t frame;
    uint32_t now_ms;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    if (LineSensor_Update() && LineSensor_GetFrame(&frame))
    {
        if (LineFollow_Update(&frame) &&
            LineFollow_GetResult(&s_latest_result))
        {
            s_has_result = true;
        }
    }

    now_ms = HAL_GetTick();
    if (!s_has_result ||
        ((uint32_t)(now_ms - s_last_report_ms) <
         TEST_LINE_FOLLOW_REPORT_PERIOD_MS))
    {
        return;
    }
    s_last_report_ms = now_ms;

    (void)BSP_Debug_Printf(
        "FOLLOW,SEQ=%lu,MASK=0x%02X,STATE=%s,POS=%d,ERR=%d,"
        "BLACK=%u,SUM=%u\r\n",
        (unsigned long)s_latest_result.sequence,
        (unsigned int)s_latest_result.black_mask,
        LineFollow_StateName(s_latest_result.state),
        (int)s_latest_result.position,
        (int)s_latest_result.error,
        (unsigned int)s_latest_result.black_count,
        (unsigned int)s_latest_result.strength_sum);
}

void Test_LineFollow_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    (void)LineSensor_Stop();
    s_initialized = false;
    (void)BSP_Debug_Printf("TEST,LINE_FOLLOW,STOP\r\n");
}

bool Test_LineFollow_IsInitialized(void)
{
    return s_initialized;
}
