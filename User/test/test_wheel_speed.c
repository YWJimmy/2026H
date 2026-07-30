#include "test_wheel_speed.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"
#include "test_config.h"

#include <stdint.h>

#define TEST_WHEEL_SPEED_STEP_HOLD_MS    3000U
#define TEST_WHEEL_SPEED_PRINT_MS        100U

#if PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_MMPS

static const int32_t s_speed_steps[] =
{
    0,
    200,
    400,
    600,
    400,
    200,
    0
};

#define TEST_WHEEL_SPEED_UNIT_NAME       "MMPS"

#elif PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_CPS

static const int32_t s_speed_steps[] =
{
    0,
    1500,
    3000,
    4500,
    3000,
    1500,
    0
};

#define TEST_WHEEL_SPEED_UNIT_NAME       "CPS"

#else
#error "test_wheel_speed.c requires a wheel speed test mode"
#endif

#define TEST_WHEEL_SPEED_STEP_COUNT \
    ((uint8_t)(sizeof(s_speed_steps) / sizeof(s_speed_steps[0])))

static bool s_initialized = false;
static bool s_finished = false;
static uint8_t s_step_index = 0U;
static uint32_t s_step_start_ms = 0U;
static uint32_t s_last_print_ms = 0U;

static bool Test_WheelSpeed_SetTarget(int32_t target)
{
#if PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_MMPS

    return Chassis_SetWheelSpeedMmps(target, target);

#else

    return Chassis_SetWheelSpeedCps(target, target);

#endif
}

static void Test_WheelSpeed_ApplyStep(void)
{
    int32_t target = s_speed_steps[s_step_index];

    (void)Test_WheelSpeed_SetTarget(target);

    (void)BSP_Debug_Printf(
        "WSPD,STEP=%u,UNIT=%s,TARGET=%ld\r\n",
        (unsigned int)s_step_index,
        TEST_WHEEL_SPEED_UNIT_NAME,
        (long)target);
}

bool Test_WheelSpeed_Init(void)
{
    s_initialized = false;
    s_finished = false;
    s_step_index = 0U;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!Chassis_Init())
    {
        (void)BSP_Debug_Printf("ERR,CHASSIS_INIT\r\n");
        return false;
    }

    if (!Chassis_Enable(true))
    {
        (void)BSP_Debug_Printf("ERR,CHASSIS_ENABLE\r\n");
        return false;
    }

    s_step_start_ms = HAL_GetTick();
    s_last_print_ms = s_step_start_ms;
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,WHEEL_SPEED,START,UNIT=%s,"
        "HOLD_MS=%lu,PRINT_MS=%lu,NOTICE=LIFT_WHEELS\r\n",
        TEST_WHEEL_SPEED_UNIT_NAME,
        (unsigned long)TEST_WHEEL_SPEED_STEP_HOLD_MS,
        (unsigned long)TEST_WHEEL_SPEED_PRINT_MS);

    Test_WheelSpeed_ApplyStep();
    return true;
}

void Test_WheelSpeed_Update(void)
{
    uint32_t now_ms;
    ChassisStatus_t status;

    BSP_DebugUart_Process();

    if ((!s_initialized) || s_finished)
    {
        return;
    }

    (void)Chassis_Update();
    now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - s_last_print_ms) >=
        TEST_WHEEL_SPEED_PRINT_MS)
    {
        s_last_print_ms = now_ms;

        if (Chassis_GetStatus(&status))
        {
            (void)BSP_Debug_Printf(
                "WSPD,SEQ=%lu,DT=%u,"
                "LTG_MM=%ld,RTG_MM=%ld,"
                "LM_MM=%ld,RM_MM=%ld,"
                "LTG_CPS=%ld,RTG_CPS=%ld,"
                "LM_CPS=%ld,RM_CPS=%ld,"
                "LRAW=%ld,RRAW=%ld,"
                "LE=%ld,RE=%ld,"
                "LI=%ld,RI=%ld,"
                "LP=%d,RP=%d,"
                "LD=%d,RD=%d,OVR=%lu\r\n",
                (unsigned long)status.control_sequence,
                (unsigned int)status.dt_ms,
                (long)status.left_target_mm_s,
                (long)status.right_target_mm_s,
                (long)status.left_measured_mm_s,
                (long)status.right_measured_mm_s,
                (long)status.left_target_cps,
                (long)status.right_target_cps,
                (long)status.left_measured_cps,
                (long)status.right_measured_cps,
                (long)status.left_raw_measured_cps,
                (long)status.right_raw_measured_cps,
                (long)status.left_error_cps,
                (long)status.right_error_cps,
                (long)status.left_integral_pwm,
                (long)status.right_integral_pwm,
                (int)status.left_pwm,
                (int)status.right_pwm,
                (int)status.left_delta,
                (int)status.right_delta,
                (unsigned long)status.timing_overrun_count);
        }
    }

    if ((uint32_t)(now_ms - s_step_start_ms) <
        TEST_WHEEL_SPEED_STEP_HOLD_MS)
    {
        return;
    }

    s_step_start_ms = now_ms;
    s_step_index++;

    if (s_step_index >= TEST_WHEEL_SPEED_STEP_COUNT)
    {
        Test_WheelSpeed_Stop();
        s_finished = true;
        return;
    }

    Test_WheelSpeed_ApplyStep();
}

void Test_WheelSpeed_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    Chassis_Stop();
    (void)Chassis_Enable(false);
    s_initialized = false;

    (void)BSP_Debug_Printf("TEST,WHEEL_SPEED,STOP\r\n");
}

bool Test_WheelSpeed_IsInitialized(void)
{
    return s_initialized;
}
