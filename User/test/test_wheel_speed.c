#include "test_wheel_speed.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "chassis_config.h"
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
#define TEST_WHEEL_SPEED_USE_CPS         0U
#define TEST_WHEEL_SPEED_MODE_ENABLED    1U

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
#define TEST_WHEEL_SPEED_USE_CPS         1U
#define TEST_WHEEL_SPEED_MODE_ENABLED    1U

#else

/*
 * 最新工程始终编译test_wheel_speed.c，即使当前选择UART巡线、
 * line_follow等其他测试。此分支保证文件在非轮速模式下仍可编译，
 * 但TestRunner不会调用轮速测试。
 */
static const int32_t s_speed_steps[] =
{
    0
};

#define TEST_WHEEL_SPEED_UNIT_NAME       "DISABLED"
#define TEST_WHEEL_SPEED_USE_CPS         0U
#define TEST_WHEEL_SPEED_MODE_ENABLED    0U

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
#if TEST_WHEEL_SPEED_MODE_ENABLED == 0U

    (void)target;
    return false;

#elif TEST_WHEEL_SPEED_USE_CPS == 1U

    return Chassis_SetWheelSpeedCps(target, target);

#else

    return Chassis_SetWheelSpeedMmps(target, target);

#endif
}

static void Test_WheelSpeed_ApplyStep(void)
{
    int32_t target = s_speed_steps[s_step_index];

    (void)Test_WheelSpeed_SetTarget(target);

    (void)BSP_Debug_Printf(
        "WSPD,STEP=%u,U=%s,T=%ld\r\n",
        (unsigned int)s_step_index,
        TEST_WHEEL_SPEED_UNIT_NAME,
        (long)target);
}

static void Test_WheelSpeed_PrintStatus(
    const ChassisStatus_t *status)
{
    /*
     * 拆成三条短消息，避免USART1调试队列单条160字节上限截断。
     */
    (void)BSP_Debug_Printf(
        "WSPDS,S=%lu,DT=%u,"
        "TMM=%ld/%ld,MMM=%ld/%ld,"
        "TC=%ld/%ld,MC=%ld/%ld,E=%ld/%ld\r\n",
        (unsigned long)status->control_sequence,
        (unsigned int)status->dt_ms,
        (long)status->left_target_mm_s,
        (long)status->right_target_mm_s,
        (long)status->left_measured_mm_s,
        (long)status->right_measured_mm_s,
        (long)status->left_target_cps,
        (long)status->right_target_cps,
        (long)status->left_measured_cps,
        (long)status->right_measured_cps,
        (long)status->left_error_cps,
        (long)status->right_error_cps);

    (void)BSP_Debug_Printf(
        "WSPDP,S=%lu,FF=%d/%d,"
        "P=%ld/%ld,I=%ld/%ld,U=%d/%d\r\n",
        (unsigned long)status->control_sequence,
        (int)status->left_feedforward_pwm,
        (int)status->right_feedforward_pwm,
        (long)status->left_proportional_pwm,
        (long)status->right_proportional_pwm,
        (long)status->left_integral_pwm,
        (long)status->right_integral_pwm,
        (int)status->left_pwm,
        (int)status->right_pwm);

    (void)BSP_Debug_Printf(
        "WSPDX,S=%lu,D=%d/%d,SAT=%u/%u,V=%u,"
        "REJ=%lu/%lu,OVR=%lu,DROP=%lu,TR=%lu\r\n",
        (unsigned long)status->control_sequence,
        (int)status->left_delta,
        (int)status->right_delta,
        status->left_output_saturated ? 1U : 0U,
        status->right_output_saturated ? 1U : 0U,
        status->encoder_sample_valid ? 1U : 0U,
        (unsigned long)status->left_encoder_reject_count,
        (unsigned long)status->right_encoder_reject_count,
        (unsigned long)status->timing_overrun_count,
        (unsigned long)BSP_DebugUart_GetDroppedCount(),
        (unsigned long)BSP_DebugUart_GetTruncatedCount());
}

bool Test_WheelSpeed_Init(void)
{
#if TEST_WHEEL_SPEED_MODE_ENABLED == 0U
    return false;
#else

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
        "TEST,WHEEL_SPEED,START,U=%s,HOLD=%lu,PRINT=%lu,"
        "KP=%ld/%ld,KI=%ld/%ld,FF=%u,LIFT=1\r\n",
        TEST_WHEEL_SPEED_UNIT_NAME,
        (unsigned long)TEST_WHEEL_SPEED_STEP_HOLD_MS,
        (unsigned long)TEST_WHEEL_SPEED_PRINT_MS,
        (long)WHEEL_SPEED_LEFT_KP_Q10,
        (long)WHEEL_SPEED_RIGHT_KP_Q10,
        (long)WHEEL_SPEED_LEFT_KI_Q10,
        (long)WHEEL_SPEED_RIGHT_KI_Q10,
        (unsigned int)WHEEL_SPEED_FEEDFORWARD_ENABLE);

    Test_WheelSpeed_ApplyStep();
    return true;
#endif
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
            Test_WheelSpeed_PrintStatus(&status);
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
