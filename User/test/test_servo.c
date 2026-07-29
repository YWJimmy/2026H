#include "test_servo.h"

#include "bsp_debug_uart.h"
#include "bsp_servo.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_SERVO_STEP_PERIOD_MS    2000U

static const uint16_t s_test_pulses_us[] =
{
    1500U,
    1450U,
    1500U,
    1550U,
    1500U
};

#define TEST_SERVO_STEP_COUNT \
    ((uint8_t)(sizeof(s_test_pulses_us) / sizeof(s_test_pulses_us[0])))

static bool s_initialized = false;
static uint8_t s_step_index = 0U;
static uint32_t s_step_start_ms = 0U;

static bool Test_Servo_ApplyStep(uint8_t step_index)
{
    uint16_t pulse_us;

    if (step_index >= TEST_SERVO_STEP_COUNT)
    {
        return false;
    }

    pulse_us = s_test_pulses_us[step_index];

    if (!BSP_Servo_SetPulseUs(pulse_us))
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "SERVO,STEP=%u,PULSE_US=%u,EN=%u\r\n",
        (unsigned int)step_index,
        (unsigned int)BSP_Servo_GetPulseUs(),
        BSP_Servo_IsEnabled() ? 1U : 0U);

    return true;
}

bool Test_Servo_Init(void)
{
    s_initialized = false;
    s_step_index = 0U;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_Servo_Init())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_INIT\r\n");
        return false;
    }

    if (!BSP_Servo_SetPulseUs(BSP_SERVO_PULSE_CENTER_US))
    {
        (void)BSP_Debug_Printf("ERR,SERVO_SET_CENTER\r\n");
        return false;
    }

    if (!BSP_Servo_Enable())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_ENABLE\r\n");
        return false;
    }

    s_step_start_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,SERVO,START,MIN_US=%u,CENTER_US=%u,MAX_US=%u,"
        "STEP_MS=%lu\r\n",
        (unsigned int)BSP_SERVO_PULSE_MIN_US,
        (unsigned int)BSP_SERVO_PULSE_CENTER_US,
        (unsigned int)BSP_SERVO_PULSE_MAX_US,
        (unsigned long)TEST_SERVO_STEP_PERIOD_MS);

    if (!Test_Servo_ApplyStep(s_step_index))
    {
        Test_Servo_Stop();
        return false;
    }

    return true;
}

void Test_Servo_Update(void)
{
    uint32_t now_ms;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - s_step_start_ms) <
        TEST_SERVO_STEP_PERIOD_MS)
    {
        return;
    }

    s_step_start_ms = now_ms;
    s_step_index++;

    if (s_step_index >= TEST_SERVO_STEP_COUNT)
    {
        s_step_index = 0U;
    }

    if (!Test_Servo_ApplyStep(s_step_index))
    {
        (void)BSP_Debug_Printf("ERR,SERVO_STEP\r\n");
        Test_Servo_Stop();
    }
}

void Test_Servo_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    BSP_Servo_Disable();
    s_initialized = false;

    (void)BSP_Debug_Printf(
        "TEST,SERVO,STOP,PULSE_US=%u,EN=%u\r\n",
        (unsigned int)BSP_Servo_GetPulseUs(),
        BSP_Servo_IsEnabled() ? 1U : 0U);
}

bool Test_Servo_IsInitialized(void)
{
    return s_initialized;
}
