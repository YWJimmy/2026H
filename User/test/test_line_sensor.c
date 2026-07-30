#include "test_line_sensor.h"

#include "bsp_debug_uart.h"
#include "line_sensor.h"
#include "stm32f4xx_hal.h"

#define TEST_LINE_SENSOR_REPORT_PERIOD_MS      100U

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;
static LineSensorFrame_t s_latest_frame;
static bool s_has_frame = false;

bool Test_LineSensor_Init(void)
{
    s_initialized = false;
    s_has_frame = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!LineSensor_Init())
    {
        (void)BSP_Debug_Printf("ERR,LINE_SENSOR_INIT\r\n");
        return false;
    }

    s_last_report_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,LINE_SENSOR,START,LOGIC_ORDER=L_TO_R,BLACK_BIT=1\r\n");

    return true;
}

void Test_LineSensor_Update(void)
{
    uint32_t now_ms;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    if (LineSensor_Update() && LineSensor_GetFrame(&s_latest_frame))
    {
        s_has_frame = true;
    }

    now_ms = HAL_GetTick();
    if (!s_has_frame ||
        ((uint32_t)(now_ms - s_last_report_ms) <
         TEST_LINE_SENSOR_REPORT_PERIOD_MS))
    {
        return;
    }
    s_last_report_ms = now_ms;

    (void)BSP_Debug_Printf(
        "LINE,SEQ=%lu,MASK=0x%02X,VALID=0x%02X\r\n",
        (unsigned long)s_latest_frame.sequence,
        (unsigned int)s_latest_frame.black_mask,
        (unsigned int)s_latest_frame.valid_mask);

    (void)BSP_Debug_Printf(
        "LINE,RAW=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned int)s_latest_frame.raw[0],
        (unsigned int)s_latest_frame.raw[1],
        (unsigned int)s_latest_frame.raw[2],
        (unsigned int)s_latest_frame.raw[3],
        (unsigned int)s_latest_frame.raw[4],
        (unsigned int)s_latest_frame.raw[5],
        (unsigned int)s_latest_frame.raw[6],
        (unsigned int)s_latest_frame.raw[7]);

    (void)BSP_Debug_Printf(
        "LINE,STR=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned int)s_latest_frame.strength[0],
        (unsigned int)s_latest_frame.strength[1],
        (unsigned int)s_latest_frame.strength[2],
        (unsigned int)s_latest_frame.strength[3],
        (unsigned int)s_latest_frame.strength[4],
        (unsigned int)s_latest_frame.strength[5],
        (unsigned int)s_latest_frame.strength[6],
        (unsigned int)s_latest_frame.strength[7]);
}

void Test_LineSensor_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    (void)LineSensor_Stop();
    s_initialized = false;
    (void)BSP_Debug_Printf("TEST,LINE_SENSOR,STOP\r\n");
}

bool Test_LineSensor_IsInitialized(void)
{
    return s_initialized;
}
