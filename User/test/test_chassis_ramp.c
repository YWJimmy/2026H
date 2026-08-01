#include "test_chassis_ramp.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "chassis_config.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_CHASSIS_RAMP_REPORT_MS       ((uint32_t)20U)
#define TEST_CHASSIS_RAMP_DEBOUNCE_MS     ((uint32_t)30U)

typedef enum
{
    TEST_PROFILE_ACTION_WAIT = 0,
    TEST_PROFILE_ACTION_SET_SPEED,
    TEST_PROFILE_ACTION_SOFT_STOP,
    TEST_PROFILE_ACTION_FAST_STOP
} TestProfileAction_t;

typedef struct
{
    TestProfileAction_t action;
    int32_t left_command_mm_s;
    int32_t right_command_mm_s;
    uint32_t hold_or_timeout_ms;
} TestChassisRampStep_t;

typedef struct
{
    GPIO_PinState raw_state;
    GPIO_PinState stable_state;
    uint32_t raw_change_ms;
} TestChassisRampKey_t;

static const TestChassisRampStep_t s_steps[] =
{
    {TEST_PROFILE_ACTION_WAIT,       0,   0,   1000U},
    {TEST_PROFILE_ACTION_SET_SPEED,  360, 360, 3000U},
    {TEST_PROFILE_ACTION_SET_SPEED,  500, 120, 2500U},
    {TEST_PROFILE_ACTION_SET_SPEED,  120, 500, 2500U},
    {TEST_PROFILE_ACTION_SOFT_STOP,  0,   0,   2500U},
    {TEST_PROFILE_ACTION_SET_SPEED,  360, 360, 3000U},
    {TEST_PROFILE_ACTION_FAST_STOP,  0,   0,   2000U}
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

static const char *Test_ChassisRamp_ActionName(
    TestProfileAction_t action)
{
    switch (action)
    {
        case TEST_PROFILE_ACTION_WAIT:
            return "WAIT";
        case TEST_PROFILE_ACTION_SET_SPEED:
            return "SET";
        case TEST_PROFILE_ACTION_SOFT_STOP:
            return "SOFT";
        case TEST_PROFILE_ACTION_FAST_STOP:
            return "FAST";
        default:
            return "UNKNOWN";
    }
}

static bool Test_ChassisRamp_ApplyStep(uint8_t index)
{
    const TestChassisRampStep_t *step;
    bool ok = true;

    if (index >= TEST_CHASSIS_RAMP_STEP_COUNT)
    {
        return false;
    }

    step = &s_steps[index];

    switch (step->action)
    {
        case TEST_PROFILE_ACTION_WAIT:
            break;

        case TEST_PROFILE_ACTION_SET_SPEED:
            ok = Chassis_SetWheelSpeedMmps(
                step->left_command_mm_s,
                step->right_command_mm_s);
            break;

        case TEST_PROFILE_ACTION_SOFT_STOP:
            ok = Chassis_RequestStop(
                CHASSIS_STOP_MODE_SOFT);
            break;

        case TEST_PROFILE_ACTION_FAST_STOP:
            ok = Chassis_RequestStop(
                CHASSIS_STOP_MODE_FAST);
            break;

        default:
            ok = false;
            break;
    }

    if (!ok)
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "CRAMP,EVENT=STEP,IDX=%u,ACT=%s,CMD=%ld/%ld,"
        "HOLD=%lu\r\n",
        (unsigned int)index,
        Test_ChassisRamp_ActionName(step->action),
        (long)step->left_command_mm_s,
        (long)step->right_command_mm_s,
        (unsigned long)step->hold_or_timeout_ms);

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

static void Test_ChassisRamp_EmergencyStop(void)
{
    Chassis_Stop();
    (void)Chassis_Enable(false);
    s_running = false;

    (void)BSP_Debug_Printf(
        "CRAMP,EVENT=KEY_EMERGENCY_STOP\r\n");
}

static bool Test_ChassisRamp_ShouldAdvance(uint32_t now_ms)
{
    const TestChassisRampStep_t *step = &s_steps[s_step_index];
    uint32_t elapsed = (uint32_t)(now_ms - s_step_start_ms);

    if ((step->action == TEST_PROFILE_ACTION_SOFT_STOP) ||
        (step->action == TEST_PROFILE_ACTION_FAST_STOP))
    {
        if (Chassis_IsMotionStopped())
        {
            return true;
        }
    }

    return elapsed >= step->hold_or_timeout_ms;
}

static void Test_ChassisRamp_CompleteSequence(void)
{
    if (!Chassis_IsMotionStopped())
    {
        Chassis_Stop();
    }

    (void)Chassis_Enable(false);
    s_running = false;

    (void)BSP_Debug_Printf(
        "CRAMP,EVENT=SEQUENCE_DONE\r\n");
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
        "TEST,CHASSIS_PROFILE_V4,READY,"
        "KEY=KEY0_PE4_ACTIVE_HIGH,PERIOD_MS=%u,MAX=%ld\r\n",
        (unsigned int)CHASSIS_CONTROL_PERIOD_MS,
        (long)CHASSIS_MAX_WHEEL_SPEED_MM_S);

    (void)BSP_Debug_Printf(
        "CRAMP,CFG=ACC:%ld,DEC:%ld,JACC:%ld,JDEC:%ld,"
        "TACC:%ld,TJ:%ld,SOFT:%ld/%ld,FAST:%ld/%ld\r\n",
        (long)CHASSIS_PROFILE_FORWARD_ACCEL_MM_S2,
        (long)CHASSIS_PROFILE_FORWARD_DECEL_MM_S2,
        (long)CHASSIS_PROFILE_FORWARD_ACCEL_JERK_MM_S3,
        (long)CHASSIS_PROFILE_FORWARD_DECEL_JERK_MM_S3,
        (long)CHASSIS_PROFILE_TURN_ACCEL_MM_S2,
        (long)CHASSIS_PROFILE_TURN_JERK_MM_S3,
        (long)CHASSIS_PROFILE_SOFT_STOP_DECEL_MM_S2,
        (long)CHASSIS_PROFILE_SOFT_STOP_JERK_MM_S3,
        (long)CHASSIS_PROFILE_FAST_STOP_DECEL_MM_S2,
        (long)CHASSIS_PROFILE_FAST_STOP_JERK_MM_S3);

    (void)BSP_Debug_Printf(
        "CRAMP,NOTICE=LIFT_WHEELS,KEY0_START_EMERGENCY_STOP\r\n");

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
            Test_ChassisRamp_EmergencyStop();
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

    if (Test_ChassisRamp_ShouldAdvance(now_ms))
    {
        s_step_index++;

        if (s_step_index >= TEST_CHASSIS_RAMP_STEP_COUNT)
        {
            Test_ChassisRamp_CompleteSequence();
            return;
        }

        s_step_start_ms = now_ms;

        if (!Test_ChassisRamp_ApplyStep(s_step_index))
        {
            Test_ChassisRamp_EmergencyStop();
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
        "CRAMP,IDX=%u,MODE=%s,CMD=%ld/%ld,RMP=%ld/%ld,"
        "MEA=%ld/%ld,F=%ld,T=%ld,AF=%ld,AT=%ld,"
        "SR=%ld,SA=%ld,PWM=%d/%d,A=%u,STOP=%u,DT=%u\r\n",
        (unsigned int)s_step_index,
        Chassis_MotionModeName(status.motion_mode),
        (long)status.left_command_mm_s,
        (long)status.right_command_mm_s,
        (long)status.left_target_mm_s,
        (long)status.right_target_mm_s,
        (long)status.left_measured_mm_s,
        (long)status.right_measured_mm_s,
        (long)status.forward_target_mm_s,
        (long)status.turn_target_mm_s,
        (long)status.forward_accel_mm_s2,
        (long)status.turn_accel_mm_s2,
        (long)status.stop_reference_mm_s,
        (long)status.stop_accel_mm_s2,
        (int)status.left_pwm,
        (int)status.right_pwm,
        status.speed_ramp_active ? 1U : 0U,
        status.motion_stopped ? 1U : 0U,
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
        Test_ChassisRamp_EmergencyStop();
    }

    s_initialized = false;
    (void)BSP_Debug_Printf(
        "TEST,CHASSIS_PROFILE_V4,STOP\r\n");
}

bool Test_ChassisRamp_IsInitialized(void)
{
    return s_initialized;
}
