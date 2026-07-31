#include "test_ball_balance.h"

#include "ball_balance_control.h"
#include "ball_balance_control_config.h"
#include "bsp_debug_uart.h"
#include "stm32f4xx_hal.h"
#include "vision.h"

#include <stddef.h>
#include <stdint.h>

#define TEST_BALL_BALANCE_PRINT_PERIOD_MS          ((uint32_t)50U)
#define TEST_BALL_BALANCE_HEARTBEAT_MS             ((uint32_t)2000U)

static bool s_initialized = false;
static bool s_vision_ok = false;

static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t s_last_event_sequence = 0U;
static uint32_t s_last_servo_error_count = 0U;
static uint32_t s_target_hold_start_ms = 0U;
static bool s_target_hold_active = false;

static int32_t Test_BallBalance_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static bool Test_BallBalance_UpdateTargetCycle(
    const BallBalanceControlStatus_t *control,
    uint32_t now_ms)
{
    int32_t next_target;

    if ((control == NULL) ||
        !control->has_target ||
        !control->vision_data_valid ||
        (Test_BallBalance_AbsI32(control->error) >
         BALL_BALANCE_DEADZONE_PX))
    {
        s_target_hold_active = false;
        return true;
    }

    if (!s_target_hold_active)
    {
        s_target_hold_start_ms = now_ms;
        s_target_hold_active = true;
        return true;
    }

    if ((uint32_t)(now_ms - s_target_hold_start_ms) <
        BALL_BALANCE_TARGET_HOLD_MS)
    {
        return true;
    }

    next_target = (control->target_x == BALL_BALANCE_TARGET_CX_A)
        ? BALL_BALANCE_TARGET_CX_B
        : BALL_BALANCE_TARGET_CX_A;

    s_target_hold_active = false;

    if (!BallBalanceControl_SetTargetX(next_target))
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,CYCLE,NEW_TARGET=%ld\r\n",
        (long)next_target);
    return true;
}

static void Test_BallBalance_PrintEvent(
    const BallBalanceControlStatus_t *control)
{
    if ((control == NULL) ||
        (control->event_sequence == s_last_event_sequence))
    {
        return;
    }

    s_last_event_sequence = control->event_sequence;

    (void)BSP_Debug_Printf(
        "BALL_BAL,EVENT=%s,PULSE=%u,SEQ=%lu\r\n",
        BallBalanceControl_EventName(control->last_event),
        (unsigned int)control->servo_pulse_us,
        (unsigned long)control->event_sequence);
}

bool Test_BallBalance_Init(void)
{
    BallBalanceControlStatus_t control;
    uint32_t now_ms;

    s_initialized = false;
    s_vision_ok = false;
    s_last_event_sequence = 0U;
    s_last_servo_error_count = 0U;
    s_target_hold_start_ms = 0U;
    s_target_hold_active = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,START,ARCH=BSP_MODULE,"
        "TARGETS=%ld/%ld,HOLD_MS=%lu,DEADZONE=%ld,KP_Q10=%ld,"
        "BRAKE_GAIN=%ld,BRAKE_BASE=%ld,BRAKE_MAX=%ld,"
        "LIMS=PUSH_%u_%u_BRAKE_%u_%u,"
        "STUCK=%ld/%lu\r\n",
        (long)BALL_BALANCE_TARGET_CX_A,
        (long)BALL_BALANCE_TARGET_CX_B,
        (unsigned long)BALL_BALANCE_TARGET_HOLD_MS,
        (long)BALL_BALANCE_DEADZONE_PX,
        (long)BALL_BALANCE_PUSH_KP_Q10,
        (long)BALL_BALANCE_BRAKE_DISTANCE_GAIN,
        (long)BALL_BALANCE_BRAKE_BASE_US,
        (long)BALL_BALANCE_BRAKE_MAX_US,
        (unsigned int)BALL_BALANCE_SERVO_PUSH_MIN_US,
        (unsigned int)BALL_BALANCE_SERVO_PUSH_MAX_US,
        (unsigned int)BALL_BALANCE_SERVO_BRAKE_MIN_US,
        (unsigned int)BALL_BALANCE_SERVO_BRAKE_MAX_US,
        (long)BALL_BALANCE_STUCK_SPEED_PX,
        (unsigned long)BALL_BALANCE_STUCK_BOOST_US);

    if (!BallBalanceControl_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,BALL_BALANCE_CONTROL_INIT\r\n");
        return false;
    }

    if (BallBalanceControl_GetStatus(&control))
    {
        (void)BSP_Debug_Printf(
            "OK,BALL_CONTROL,PULSE=%u,EN=%u,MODE=%s\r\n",
            (unsigned int)control.servo_pulse_us,
            control.servo_enabled ? 1U : 0U,
            BallBalanceControl_ModeName(control.mode));
    }

    /*
     * 保持f426配置：Vision初始化失败为非致命错误。
     * 此时钢球控制模块保持舵机水平，便于分别排查。
     */
    if (!Vision_Init())
    {
        (void)BSP_Debug_Printf("ERR,VISION_INIT\r\n");
    }
    else
    {
        s_vision_ok = true;
        (void)BSP_Debug_Printf("OK,VISION_INIT\r\n");
    }

    now_ms = HAL_GetTick();
    s_last_print_ms = now_ms;
    s_last_heartbeat_ms = now_ms;
    s_initialized = true;
    return true;
}

void Test_BallBalance_Update(void)
{
    VisionStatus_t vision_status;
    BallBalanceControlStatus_t control;
    bool have_vision_status = false;
    bool have_control_status;
    bool control_update_ok = true;
    uint32_t now_ms;

    if (!s_initialized)
    {
        return;
    }

    BSP_DebugUart_Process();

    if (s_vision_ok)
    {
        Vision_Update();
        have_vision_status =
            Vision_GetStatus(&vision_status);

        if (have_vision_status)
        {
            control_update_ok =
                BallBalanceControl_Update(
                    &vision_status);
        }
    }

    have_control_status =
        BallBalanceControl_GetStatus(&control);
    now_ms = HAL_GetTick();

    if (have_control_status)
    {
        if (!Test_BallBalance_UpdateTargetCycle(
                &control,
                now_ms))
        {
            (void)BSP_Debug_Printf(
                "ERR,BALL_TARGET_SWITCH\r\n");
        }
        else
        {
            /* Refresh target/error after a possible cycle switch. */
            have_control_status =
                BallBalanceControl_GetStatus(&control);
        }

        if (!control_update_ok &&
            (control.servo_error_count !=
             s_last_servo_error_count))
        {
            s_last_servo_error_count =
                control.servo_error_count;

            (void)BSP_Debug_Printf(
                "ERR,BALL_SERVO_CMD,COUNT=%lu,"
                "MODE=%s,EVENT=%s\r\n",
                (unsigned long)control.servo_error_count,
                BallBalanceControl_ModeName(control.mode),
                BallBalanceControl_EventName(
                    control.last_event));
        }

        Test_BallBalance_PrintEvent(&control);
    }
    else if (!control_update_ok)
    {
        /*
         * 正常情况下控制模块已初始化，状态应可读取。
         * 保留兜底日志，避免Update失败被静默忽略。
         */
        (void)BSP_Debug_Printf(
            "ERR,BALL_CONTROL_UPDATE,STATUS=UNAVAILABLE\r\n");
    }

    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >=
        TEST_BALL_BALANCE_HEARTBEAT_MS)
    {
        s_last_heartbeat_ms = now_ms;

        if (s_vision_ok && have_vision_status &&
            have_control_status)
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,HB,SEQ=%lu,VF=%lu,IF=%lu,"
                "PO=%lu,UE=%lu,VALID=%u,MODE=%s\r\n",
                (unsigned long)vision_status.sequence,
                (unsigned long)vision_status.valid_frame_count,
                (unsigned long)vision_status.invalid_frame_count,
                (unsigned long)vision_status.protocol_overflow_count,
                (unsigned long)vision_status.uart_error_count,
                vision_status.data_valid ? 1U : 0U,
                BallBalanceControl_ModeName(control.mode));
        }
        else if (have_control_status)
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,HB,VISION=0,PULSE=%u,MODE=%s\r\n",
                (unsigned int)control.servo_pulse_us,
                BallBalanceControl_ModeName(control.mode));
        }
    }

    if (have_control_status &&
        ((uint32_t)(now_ms - s_last_print_ms) >=
         TEST_BALL_BALANCE_PRINT_PERIOD_MS))
    {
        s_last_print_ms = now_ms;

        if (control.has_target)
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,CX=%ld,TARGET=%ld,ERR=%ld,SPD=%ld,D=%ld,"
                "MODE=%s,BOOST=%lu,PULSE=%u\r\n",
                (long)control.center_x,
                (long)control.target_x,
                (long)control.error,
                (long)control.speed,
                (long)control.delta_us,
                BallBalanceControl_ModeName(control.mode),
                (unsigned long)control.stuck_boost_us,
                (unsigned int)control.servo_pulse_us);
        }
        else
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,NO_TARGET,LOST=%lu,MODE=%s,PULSE=%u\r\n",
                (unsigned long)control.lost_frames,
                BallBalanceControl_ModeName(control.mode),
                (unsigned int)control.servo_pulse_us);
        }
    }
}

void Test_BallBalance_Stop(void)
{
    BallBalanceControlStatus_t control;
    uint16_t pulse_us =
        BALL_BALANCE_SERVO_CENTER_US;

    if (!s_initialized)
    {
        return;
    }

    if (s_vision_ok)
    {
        Vision_Stop();
    }

    if (BallBalanceControl_GetStatus(&control))
    {
        pulse_us = control.servo_pulse_us;
    }

    BallBalanceControl_Stop();

    s_vision_ok = false;
    s_initialized = false;

    (void)BSP_Debug_Printf(
        "BALL_BAL,STOP,PREV_PULSE=%u,CENTER=%u\r\n",
        (unsigned int)pulse_us,
        (unsigned int)BALL_BALANCE_SERVO_CENTER_US);
}

bool Test_BallBalance_IsInitialized(void)
{
    return s_initialized;
}
