#include "test_chassis_ramp.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "chassis_config.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_CHASSIS_RAMP_REPORT_MS       ((uint32_t)10U)
#define TEST_CHASSIS_RAMP_DEBOUNCE_MS     ((uint32_t)30U)

typedef struct
{
    int32_t left_command_mm_s;
    int32_t right_command_mm_s;
    uint32_t hold_ms;
} TestChassisRampStep_t;

typedef struct
{
    GPIO_PinState raw_state;
    GPIO_PinState stable_state;
    uint32_t raw_change_ms;
} TestChassisRampKey_t;

static const TestChassisRampStep_t s_steps[] =
{
    {0,   0,   1000U},
    {360, 360, 2000U},
    {500, 120, 2000U},
    {120, 500, 2000U},
    {0,   0,   2000U}
};

#define TEST_CHASSIS_RAMP_STEP_COUNT \
    ((uint8_t)(sizeof(s_steps) / sizeof(s_steps[0])))

static bool s_initialized = false;
static bool s_running = false;
static uint8_t s_step_index = 0U;
static uint32_t s_step_start_ms = 0U;
static uint32_t s_last_report_ms = 0U;
static TestChassisRampKey_t s_key;

static void Test_ChassisRamp_InitKey(void)
{
    GPIO_PinState current =
        HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);

    s_key.raw_state = current;
    s_key.stable_state = current;
    s_key.raw_change_ms = HAL_GetTick();
}

static bool Test_ChassisRamp_KeyPressed(uint32_t now_ms)
{
    GPIO_PinState raw =
        HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);

    if (raw != s_key.raw_state)
    {
        s_key.raw_state = raw;
        s_key.raw_change_ms = now_ms;
    }

    if ((raw != s_key.stable_state) &&
        ((uint32_t)(now_ms - s_key.raw_change_ms) >=
         TEST_CHASSIS_RAMP_DEBOUNCE_MS))
    {
        s_key.stable_state = raw;

        if (s_key.stable_state == GPIO_PIN_SET)
        {
            return true;
        }
    }

    return false;
}

static bool Test_ChassisRamp_ApplyStep(uint8_t index)
{
    if (index >= TEST_CHASSIS_RAMP_STEP_COUNT)
    {
        return false;
    }

    if (!Chassis_SetWheelSpeedMmps(
            s_steps[index].left_command_mm_s,
            s_steps[index].right_command_mm_s))
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "CRAMP,EVENT=STEP,IDX=%u,CMD=%ld/%ld,HOLD=%lu\r\n",
        (unsigned int)index,
        (long)s_steps[index].left_command_mm_s,
        (long)s_steps[index].right_command_mm_s,
        (unsigned long)s_steps[index].hold_ms);

    return true;
}

static bool Test_ChassisRamp_Start(uint32_t now_ms)
{
    if (!Chassis_Enable(true))
    {
        (void)BSP_Debug_Printf("ERR,CHASSIS_ENABLE\r\n");
        return false;
    }

    s_step_index = 0U;
    s_step_start_ms = now_ms;
    s_last_report_ms = now_ms;

    if (!Test_ChassisRamp_ApplyStep(s_step_index))
    {
        Chassis_Stop();
        (void)Chassis_Enable(false);
        return false;
    }

    s_running = true;
    (void)BSP_Debug_Printf("CRAMP,EVENT=KEY_START\r\n");
    return true;
}

static void Test_ChassisRamp_StopRun(void)
{
    Chassis_Stop();
    (void)Chassis_Enable(false);
    s_running = false;

    (void)BSP_Debug_Printf(
        "CRAMP,EVENT=KEY_STOP,MODE=BRAKE\r\n");
}

bool Test_ChassisRamp_Init(void)
{
    s_initialized = false;
    s_running = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!Chassis_Init())
    {
        (void)BSP_Debug_Printf("ERR,CHASSIS_INIT\r\n");
        return false;
    }

    Test_ChassisRamp_InitKey();
    s_last_report_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,CHASSIS_RAMP,READY,KEY=KEY0_PE4_ACTIVE_HIGH,"
        "PERIOD_MS=%u,ACC=%ld,DEC=%ld,TURN=%ld,MAX=%ld\r\n",
        (unsigned int)CHASSIS_CONTROL_PERIOD_MS,
        (long)CHASSIS_RAMP_FORWARD_ACCEL_MM_S2,
        (long)CHASSIS_RAMP_FORWARD_DECEL_MM_S2,
        (long)CHASSIS_RAMP_TURN_SLEW_MM_S2,
        (long)CHASSIS_MAX_WHEEL_SPEED_MM_S);

    (void)BSP_Debug_Printf(
        "CRAMP,NOTICE=LIFT_WHEELS,KEY0_START_STOP\r\n");

    return true;
}

void Test_ChassisRamp_Update(void)
{
    ChassisStatus_t status;
    uint32_t now_ms;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if (Test_ChassisRamp_KeyPressed(now_ms))
    {
        if (s_running)
        {
            Test_ChassisRamp_StopRun();
        }
        else
        {
            (void)Test_ChassisRamp_Start(now_ms);
        }
    }

    if (!s_running)
    {
        return;
    }

    (void)Chassis_Update();

    if ((uint32_t)(now_ms - s_step_start_ms) >=
        s_steps[s_step_index].hold_ms)
    {
        s_step_index++;

        if (s_step_index >= TEST_CHASSIS_RAMP_STEP_COUNT)
        {
            Test_ChassisRamp_StopRun();
            (void)BSP_Debug_Printf(
                "CRAMP,EVENT=SEQUENCE_DONE\r\n");
            return;
        }

        s_step_start_ms = now_ms;

        if (!Test_ChassisRamp_ApplyStep(s_step_index))
        {
            Test_ChassisRamp_StopRun();
            return;
        }
    }

    if ((uint32_t)(now_ms - s_last_report_ms) <
        TEST_CHASSIS_RAMP_REPORT_MS)
    {
        return;
    }

    s_last_report_ms = now_ms;

    if (!Chassis_GetStatus(&status))
    {
        return;
    }

    (void)BSP_Debug_Printf(
        "CRAMP,IDX=%u,CMD=%ld/%ld,RMP=%ld/%ld,"
        "MEA=%ld/%ld,PWM=%d/%d,A=%u,DT=%u\r\n",
        (unsigned int)s_step_index,
        (long)status.left_command_mm_s,
        (long)status.right_command_mm_s,
        (long)status.left_target_mm_s,
        (long)status.right_target_mm_s,
        (long)status.left_measured_mm_s,
        (long)status.right_measured_mm_s,
        (int)status.left_pwm,
        (int)status.right_pwm,
        status.speed_ramp_active ? 1U : 0U,
        (unsigned int)status.dt_ms);
}

void Test_ChassisRamp_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    if (s_running)
    {
        Test_ChassisRamp_StopRun();
    }

    s_initialized = false;
    (void)BSP_Debug_Printf("TEST,CHASSIS_RAMP,STOP\r\n");
}

bool Test_ChassisRamp_IsInitialized(void)
{
    return s_initialized;
}
