#include "test_servo.h"

#include "bsp_debug_uart.h"
#include "bsp_servo.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

/*
 * 舵机到达一个极限后保持的时间。
 * 可按需要修改，单位为 ms。
 */
#define TEST_SERVO_LIMIT_HOLD_MS     2000U

typedef enum
{
    TEST_SERVO_TARGET_MIN = 0,
    TEST_SERVO_TARGET_MAX
} TestServoTarget_t;

static bool s_initialized = false;
static TestServoTarget_t s_target = TEST_SERVO_TARGET_MIN;
static uint32_t s_last_switch_ms = 0U;

static bool Test_Servo_ApplyTarget(TestServoTarget_t target)
{
    uint16_t pulse_us;
    const char *target_name;

    if (target == TEST_SERVO_TARGET_MIN)
    {
        pulse_us = BSP_SERVO_PULSE_MIN_US;
        target_name = "MIN";
    }
    else
    {
        pulse_us = BSP_SERVO_PULSE_MAX_US;
        target_name = "MAX";
    }

    if (!BSP_Servo_SetPulseUs(pulse_us))
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "SERVO,LIMIT=%s,PULSE_US=%u,EN=%u\r\n",
        target_name,
        (unsigned int)BSP_Servo_GetPulseUs(),
        BSP_Servo_IsEnabled() ? 1U : 0U);

    return true;
}

bool Test_Servo_Init(void)
{
    s_initialized = false;
    s_target = TEST_SERVO_TARGET_MIN;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_Servo_Init())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_INIT\r\n");
        return false;
    }

    /*
     * 先保存最小极限，再启动 PWM。
     * Enable() 后将立即输出 1000 us。
     */
    if (!BSP_Servo_SetPulseUs(BSP_SERVO_PULSE_MIN_US))
    {
        (void)BSP_Debug_Printf("ERR,SERVO_SET_MIN\r\n");
        return false;
    }

    if (!BSP_Servo_Enable())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_ENABLE\r\n");
        return false;
    }

    s_last_switch_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,SERVO,LIMIT_START,MIN_US=%u,MAX_US=%u,"
        "HOLD_MS=%lu\r\n",
        (unsigned int)BSP_SERVO_PULSE_MIN_US,
        (unsigned int)BSP_SERVO_PULSE_MAX_US,
        (unsigned long)TEST_SERVO_LIMIT_HOLD_MS);

    if (!Test_Servo_ApplyTarget(s_target))
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

    if ((uint32_t)(now_ms - s_last_switch_ms) <
        TEST_SERVO_LIMIT_HOLD_MS)
    {
        return;
    }

    /*
     * 使用累加方式更新时间基准，减小主循环短暂延迟导致的长期漂移。
     */
    s_last_switch_ms += TEST_SERVO_LIMIT_HOLD_MS;

    if (s_target == TEST_SERVO_TARGET_MIN)
    {
        s_target = TEST_SERVO_TARGET_MAX;
    }
    else
    {
        s_target = TEST_SERVO_TARGET_MIN;
    }

    if (!Test_Servo_ApplyTarget(s_target))
    {
        (void)BSP_Debug_Printf("ERR,SERVO_LIMIT_SWITCH\r\n");
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
