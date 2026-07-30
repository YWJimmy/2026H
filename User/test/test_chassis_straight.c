#include "test_chassis_straight.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_CHASSIS_STRAIGHT_SPEED_MM_S    400
#define TEST_CHASSIS_STRAIGHT_RUN_MS        5000U
#define TEST_CHASSIS_STRAIGHT_PRINT_MS      100U
#define TEST_CHASSIS_SETTLE_MS              1000U

typedef enum
{
    TEST_CHASSIS_STATE_SETTLE = 0,
    TEST_CHASSIS_STATE_RUN,
    TEST_CHASSIS_STATE_STOPPED
} TestChassisState_t;

static bool s_initialized = false;
static TestChassisState_t s_state = TEST_CHASSIS_STATE_SETTLE;
static uint32_t s_state_start_ms = 0U;
static uint32_t s_last_print_ms = 0U;

bool Test_ChassisStraight_Init(void)
{
    s_initialized = false;
    s_state = TEST_CHASSIS_STATE_SETTLE;

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

    (void)Chassis_SetWheelSpeedMmps(0, 0);

    s_state_start_ms = HAL_GetTick();
    s_last_print_ms = s_state_start_ms;
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,CHASSIS_STRAIGHT,START,SPEED_MM_S=%d,RUN_MS=%lu\r\n",
        TEST_CHASSIS_STRAIGHT_SPEED_MM_S,
        (unsigned long)TEST_CHASSIS_STRAIGHT_RUN_MS);

    return true;
}

void Test_ChassisStraight_Update(void)
{
    uint32_t now_ms;
    ChassisStatus_t status;
    int32_t total_diff;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    (void)Chassis_Update();
    now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - s_last_print_ms) >=
        TEST_CHASSIS_STRAIGHT_PRINT_MS)
    {
        s_last_print_ms = now_ms;

        if (Chassis_GetStatus(&status))
        {
            total_diff = status.left_total - status.right_total;

            (void)BSP_Debug_Printf(
                "CSTRAIGHT,STATE=%u,SEQ=%lu,"
                "LM=%ld,RM=%ld,LP=%d,RP=%d,"
                "LT=%ld,RT=%ld,DIFF=%ld,OVR=%lu\r\n",
                (unsigned int)s_state,
                (unsigned long)status.control_sequence,
                (long)status.left_measured_mm_s,
                (long)status.right_measured_mm_s,
                (int)status.left_pwm,
                (int)status.right_pwm,
                (long)status.left_total,
                (long)status.right_total,
                (long)total_diff,
                (unsigned long)status.timing_overrun_count);
        }
    }

    if (s_state == TEST_CHASSIS_STATE_SETTLE)
    {
        if ((uint32_t)(now_ms - s_state_start_ms) >=
            TEST_CHASSIS_SETTLE_MS)
        {
            s_state = TEST_CHASSIS_STATE_RUN;
            s_state_start_ms = now_ms;

            (void)Chassis_SetWheelSpeedMmps(
                TEST_CHASSIS_STRAIGHT_SPEED_MM_S,
                TEST_CHASSIS_STRAIGHT_SPEED_MM_S);
        }

        return;
    }

    if (s_state == TEST_CHASSIS_STATE_RUN)
    {
        if ((uint32_t)(now_ms - s_state_start_ms) >=
            TEST_CHASSIS_STRAIGHT_RUN_MS)
        {
            Chassis_Stop();
            s_state = TEST_CHASSIS_STATE_STOPPED;
            s_state_start_ms = now_ms;

            (void)BSP_Debug_Printf(
                "TEST,CHASSIS_STRAIGHT,RUN_COMPLETE\r\n");
        }

        return;
    }

    if ((uint32_t)(now_ms - s_state_start_ms) >= 500U)
    {
        Test_ChassisStraight_Stop();
    }
}

void Test_ChassisStraight_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    Chassis_Stop();
    (void)Chassis_Enable(false);
    s_initialized = false;

    (void)BSP_Debug_Printf(
        "TEST,CHASSIS_STRAIGHT,STOP\r\n");
}

bool Test_ChassisStraight_IsInitialized(void)
{
    return s_initialized;
}
