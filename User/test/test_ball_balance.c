#include "test_ball_balance.h"

#include "ball_position_action.h"
#include "ball_motion_estimator_config.h"
#include "bsp_debug_uart.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_BALL_ACTION_PRINT_PERIOD_MS       ((uint32_t)100U)
#define TEST_BALL_ACTION_HOLD_MS               ((uint32_t)5000U)
#define TEST_BALL_TARGET_POS_X                  ((int32_t)(BALL_CALIBRATION_ORIGIN_PX + BALL_CALIBRATION_POS_SPAN_PX))
#define TEST_BALL_TARGET_NEG_X                  ((int32_t)(BALL_CALIBRATION_ORIGIN_PX - BALL_CALIBRATION_NEG_SPAN_PX))

static bool s_initialized = false;
static int32_t s_target_x = TEST_BALL_TARGET_POS_X;
static uint32_t s_last_print_ms = 0U;
static uint32_t s_reached_ms = 0U;
static bool s_holding = false;

static bool TestBallAction_StartTarget(
    int32_t target_x,
    uint32_t now_ms,
    bool first_start)
{
    BallPositionActionCommand_t command;

    BallPositionAction_DefaultCommand(&command, target_x);
    if (first_start)
    {
        return BallPositionAction_Start(&command, now_ms);
    }
    return BallPositionAction_Retarget(&command, now_ms);
}

bool Test_BallBalance_Init(void)
{
    uint32_t now_ms;

    if (!BSP_DebugUart_Init() || !BallPositionAction_Init())
    {
        return false;
    }
    now_ms = HAL_GetTick();
    if (!TestBallAction_StartTarget(s_target_x, now_ms, true))
    {
        return false;
    }

    s_last_print_ms = now_ms;
    s_reached_ms = 0U;
    s_holding = false;
    s_initialized = true;
    (void)BSP_Debug_Printf(
        "BALL_ACTION,START=1,TARGET_X=%ld,CHAIN=VISION>ESTIMATOR>PD_FF>DYNAMICS>SERVO\r\n",
        (long)s_target_x);
    return true;
}

void Test_BallBalance_Update(void)
{
    BallPositionActionResult_t result;
    BallPositionActionStatus_t status;
    uint32_t now_ms;

    if (!s_initialized)
    {
        return;
    }
    BSP_DebugUart_Process();
    now_ms = HAL_GetTick();
    result = BallPositionAction_Update(now_ms);
    if ((result == BALL_POSITION_ACTION_RESULT_FAULT) ||
        !BallPositionAction_GetStatus(&status))
    {
        (void)BSP_Debug_Printf(
            "ERR,BALL_ACTION,DETAIL=%lu\r\n",
            (unsigned long)BallPositionAction_GetFaultDetail());
        BallPositionAction_ForceSafeStop();
        s_initialized = false;
        return;
    }

    if ((uint32_t)(now_ms - s_last_print_ms) >=
        TEST_BALL_ACTION_PRINT_PERIOD_MS)
    {
        s_last_print_ms = now_ms;
        (void)BSP_Debug_Printf(
            "BALL_ACTION,CX=%ld,XMM=%ld,TMM=%ld,ERRMM=%ld,VMM=%ld,A=%ld,FF=%ld,ANG=%ld,PULSE=%u,SCORE=%u,ACC=%lu,REJ=%lu,MODE=%s\r\n",
            (long)status.center_x,
            (long)status.position_mm,
            (long)status.target_mm,
            (long)status.error_mm,
            (long)status.velocity_mm_s,
            (long)status.estimated_accel_mm_s2,
            (long)status.chassis_ff_accel_mm_s2,
            (long)status.platform_angle_mrad,
            (unsigned int)status.servo_pulse_us,
            (unsigned int)status.confidence_milli,
            (unsigned long)status.accepted_frames,
            (unsigned long)status.rejected_frames,
            BallPositionAction_StateName(status.state));
    }

    if ((result == BALL_POSITION_ACTION_RESULT_REACHED) &&
        !s_holding)
    {
        s_holding = true;
        s_reached_ms = now_ms;
    }
    if (s_holding &&
        ((uint32_t)(now_ms - s_reached_ms) >=
         TEST_BALL_ACTION_HOLD_MS))
    {
        s_target_x =
            (s_target_x == TEST_BALL_TARGET_POS_X) ?
            TEST_BALL_TARGET_NEG_X :
            TEST_BALL_TARGET_POS_X;
        if (!TestBallAction_StartTarget(
                s_target_x,
                now_ms,
                false))
        {
            BallPositionAction_ForceSafeStop();
            s_initialized = false;
            return;
        }
        s_holding = false;
        (void)BSP_Debug_Printf(
            "BALL_ACTION,RETARGET_X=%ld\r\n",
            (long)s_target_x);
    }
}

void Test_BallBalance_Stop(void)
{
    BallPositionAction_ForceSafeStop();
    s_initialized = false;
}
