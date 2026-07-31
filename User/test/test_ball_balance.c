#include "test_ball_balance.h"

#include "bsp_debug_uart.h"
#include "bsp_servo.h"
#include "vision.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdlib.h>

/*
 * Task 3 one-shot sequence:
 *   O -> +5 cm -> -5 cm -> hold
 *
 * Coordinate calibration inherited from 0ebde21/7a35ce0:
 *   O = 653, +5 cm = 869, -5 cm = 447.
 * Pulse increase moves the ball toward larger cx.
 */
#define BALL_TARGET_CX_O                 653
#define BALL_TARGET_CX_P5                869
#define BALL_TARGET_CX_N5                447

/* Keep the proven 7a35ce0 neutral and travel pulse baseline. */
#define SERVO_NEUTRAL_US                1650U
#define SERVO_RIGHT_CRUISE_US           1750U
#define SERVO_LEFT_CRUISE_US            1550U
#define SERVO_TURN_BRAKE_US             1420U
#define SERVO_MIN_US                    1300U
#define SERVO_MAX_US                    2000U

/* Time budget: the official motion must finish within 5 seconds. */
#define BALL_RUN_LIMIT_MS               5000U
#define BALL_CENTER_READY_MAX_MS         300U
#define BALL_TURN_BRAKE_MAX_MS           450U

/* Position/speed windows in camera pixels and pixels per valid frame. */
#define BALL_CENTER_READY_ERR_PX          32
#define BALL_CENTER_READY_SPEED_PX         3
#define BALL_CENTER_READY_FRAMES           3U

#define BALL_POS5_SWITCH_ERR_PX           30
#define BALL_POS5_BRAKE_BASE_PX           38
#define BALL_POS5_BRAKE_SPEED_GAIN         4

#define BALL_NEG5_CAPTURE_ERR_PX          43
#define BALL_NEG5_BRAKE_BASE_PX           70
#define BALL_NEG5_BRAKE_SPEED_GAIN         4

/* Final acceptance is evaluated in physical millimetres. */
#define BALL_TARGET_N5_MM                 (-50)
#define BALL_FINAL_TOLERANCE_MM             10
#define BALL_FINAL_SPEED_PX                 2

/* Final capture: brake velocity first, then correct position. */
#define BALL_CAPTURE_BRAKE_BASE_US        120
#define BALL_CAPTURE_BRAKE_SPEED_GAIN       7
#define BALL_CAPTURE_BRAKE_MAX_US         285
#define BALL_SETTLE_CORR_MIN_US            60
#define BALL_SETTLE_CORR_MAX_US           140
#define BALL_SETTLE_STUCK_WAIT              5U
#define BALL_SETTLE_KICK_US                180
#define BALL_SETTLE_KICK_FRAMES             3U

/*
 * PASS time is latched on the first valid stop, but the servo must remain in a
 * low-amplitude closed loop afterwards. A fixed neutral pulse lets the ball
 * drift on a slightly tilted rod, so the finished state learns a small static
 * hold bias and damps any new motion without reopening the task result.
 */
#define BALL_FINISH_HOLD_INNER_MM             5
#define BALL_FINISH_HOLD_BIAS_STEP_US          3
#define BALL_FINISH_HOLD_BIAS_MAX_US          90
#define BALL_FINISH_HOLD_POS_GAIN_US           3
#define BALL_FINISH_HOLD_POS_MAX_US           36
#define BALL_FINISH_HOLD_SPEED_GAIN_US         8
#define BALL_FINISH_HOLD_SPEED_MAX_US         48
#define BALL_FINISH_HOLD_TOTAL_MAX_US        110
#define BALL_FINISH_HOLD_SLEW_US              18

#define BALL_LOST_TIMEOUT_FRAMES           10U
#define BALL_PRINT_PERIOD_MS               50U
#define BALL_HEARTBEAT_MS                1000U

typedef enum
{
    BALL_STATE_WAIT_CENTER = 0,
    BALL_STATE_TO_POS5,
    BALL_STATE_TURN_BRAKE,
    BALL_STATE_TO_NEG5,
    BALL_STATE_SETTLE_NEG5,
    BALL_STATE_FINISHED,
    BALL_STATE_TIMEOUT
} BallTaskState_t;

static bool s_initialized = false;
static bool s_vision_ok = false;
static bool s_result_done = false;
static bool s_result_passed = false;

static BallTaskState_t s_state = BALL_STATE_WAIT_CENTER;
static uint16_t s_target_cx = BALL_TARGET_CX_O;
static uint16_t s_servo_pulse = SERVO_NEUTRAL_US;
static const char *s_control_mode = "HOLD";

static uint32_t s_task_start_ms = 0U;
/* Automatic OLED timer starts at the actual O-point departure. */
static uint32_t s_motion_start_ms = 0U;
static bool s_motion_timer_started = false;
static uint32_t s_state_start_ms = 0U;
static uint32_t s_pos5_reached_ms = 0U;
static uint32_t s_result_elapsed_ms = 0U;
static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t s_last_vision_sequence = 0U;
static uint32_t s_lost_frames = 0U;

static int32_t s_last_cx = 0;
static int32_t s_last_x_mm = 0;
static int32_t s_last_error_mm = 0;
static int32_t s_prev_cx = 0;
static int32_t s_raw_speed = 0;
static int32_t s_speed = 0;
static int32_t s_last_error = 0;
static bool s_has_target = false;

static uint32_t s_center_ready_frames = 0U;
static uint32_t s_turn_zero_frames = 0U;
static uint32_t s_final_stable_frames = 0U;
static uint32_t s_settle_stuck_frames = 0U;
static uint32_t s_settle_kick_frames = 0U;
static int32_t s_settle_kick_dir = 0;
static int32_t s_finish_hold_bias_us = 0;

static int32_t s_pos5_peak_cx = BALL_TARGET_CX_O;
static int32_t s_neg5_min_cx = BALL_TARGET_CX_O;

static int32_t Ball_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t Ball_Clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint16_t Ball_ClampPulse(int32_t pulse)
{
    return (uint16_t)Ball_Clamp32(
        pulse,
        (int32_t)SERVO_MIN_US,
        (int32_t)SERVO_MAX_US);
}

static const char *Ball_StateName(BallTaskState_t state)
{
    switch (state)
    {
        case BALL_STATE_WAIT_CENTER:
            return "WAIT_O";
        case BALL_STATE_TO_POS5:
            return "TO_POS5";
        case BALL_STATE_TURN_BRAKE:
            return "TURN_BRAKE";
        case BALL_STATE_TO_NEG5:
            return "TO_NEG5";
        case BALL_STATE_SETTLE_NEG5:
            return "SETTLE_NEG5";
        case BALL_STATE_FINISHED:
            return "FINISHED";
        case BALL_STATE_TIMEOUT:
        default:
            return "TIMEOUT";
    }
}

static void Ball_SetPulse(uint16_t pulse, const char *mode)
{
    pulse = Ball_ClampPulse((int32_t)pulse);
    if (pulse != s_servo_pulse)
    {
        if (BSP_Servo_SetPulseUs(pulse))
        {
            s_servo_pulse = pulse;
        }
    }
    s_control_mode = mode;
}

static void Ball_SetPulseSigned(int32_t pulse, const char *mode)
{
    Ball_SetPulse(Ball_ClampPulse(pulse), mode);
}

static void Ball_SlewPulse(int32_t target_pulse, int32_t step_us, const char *mode)
{
    int32_t current = (int32_t)s_servo_pulse;
    int32_t delta = target_pulse - current;

    if (delta > step_us)
    {
        target_pulse = current + step_us;
    }
    else if (delta < -step_us)
    {
        target_pulse = current - step_us;
    }

    Ball_SetPulseSigned(target_pulse, mode);
}

static void Ball_ResetSettleCounters(void)
{
    s_final_stable_frames = 0U;
    s_settle_stuck_frames = 0U;
    s_settle_kick_frames = 0U;
    s_settle_kick_dir = 0;
}

static void Ball_EnterState(BallTaskState_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_start_ms = now_ms;
    s_turn_zero_frames = 0U;

    if (state == BALL_STATE_TO_POS5)
    {
        s_target_cx = BALL_TARGET_CX_P5;
        s_pos5_peak_cx = s_last_cx;
    }
    else if ((state == BALL_STATE_TURN_BRAKE) ||
             (state == BALL_STATE_TO_NEG5) ||
             (state == BALL_STATE_SETTLE_NEG5) ||
             (state == BALL_STATE_FINISHED))
    {
        s_target_cx = BALL_TARGET_CX_N5;
        if (state == BALL_STATE_SETTLE_NEG5)
        {
            Ball_ResetSettleCounters();
        }
    }
    else
    {
        s_target_cx = BALL_TARGET_CX_O;
    }

    s_last_error = (int32_t)s_target_cx - s_last_cx;

    (void)BSP_Debug_Printf(
        "BALL_BAL,STATE_CHANGE,STATE=%s,TARGET=%u,CX=%ld,SPD=%+ld,T=%lums\r\n",
        Ball_StateName(state),
        (unsigned int)s_target_cx,
        (long)s_last_cx,
        (long)s_speed,
        (unsigned long)(now_ms - s_task_start_ms));
}

static uint32_t Ball_GetMotionElapsedMs(uint32_t now_ms)
{
    if (!s_motion_timer_started)
    {
        return 0U;
    }

    return (uint32_t)(now_ms - s_motion_start_ms);
}

static void Ball_ReportResult(bool passed, const char *reason, uint32_t now_ms)
{
    int32_t pos5_overshoot_px;
    int32_t neg5_overshoot_px;

    if (s_result_done)
    {
        return;
    }

    s_result_done = true;
    s_result_passed = passed;
    s_result_elapsed_ms = Ball_GetMotionElapsedMs(now_ms);

    pos5_overshoot_px = s_pos5_peak_cx - BALL_TARGET_CX_P5;
    if (pos5_overshoot_px < 0)
    {
        pos5_overshoot_px = 0;
    }

    neg5_overshoot_px = BALL_TARGET_CX_N5 - s_neg5_min_cx;
    if (neg5_overshoot_px < 0)
    {
        neg5_overshoot_px = 0;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,RESULT=%s,REASON=%s,TOTAL_MS=%lu,POS5_MS=%lu,"
        "FINAL_CX=%ld,FINAL_ERR_PX=%ld,FINAL_X_MM=%ld,FINAL_ERR_MM=%ld,"
        "FINAL_SPD=%ld,"
        "POS5_OVER_PX=%ld,NEG5_OVER_PX=%ld,PULSE=%u\r\n",
        passed ? "PASS" : "FAIL",
        reason,
        (unsigned long)s_result_elapsed_ms,
        (unsigned long)s_pos5_reached_ms,
        (long)s_last_cx,
        (long)s_last_error,
        (long)s_last_x_mm,
        (long)s_last_error_mm,
        (long)s_speed,
        (long)pos5_overshoot_px,
        (long)neg5_overshoot_px,
        (unsigned int)s_servo_pulse);
}

static void Ball_ControlWaitCenter(uint32_t now_ms)
{
    int32_t correction;

    if ((Ball_Abs32(s_last_error) <= BALL_CENTER_READY_ERR_PX) &&
        (Ball_Abs32(s_speed) <= BALL_CENTER_READY_SPEED_PX))
    {
        Ball_SetPulse(SERVO_NEUTRAL_US, "CENTER_READY");
        s_center_ready_frames++;
    }
    else
    {
        correction = s_last_error - (s_speed * 3);
        correction = Ball_Clamp32(correction, -100, 100);
        Ball_SetPulseSigned((int32_t)SERVO_NEUTRAL_US + correction, "CENTERING");
        s_center_ready_frames = 0U;
    }

    if ((s_center_ready_frames >= BALL_CENTER_READY_FRAMES) ||
        ((uint32_t)(now_ms - s_task_start_ms) >= BALL_CENTER_READY_MAX_MS))
    {
        /*
         * Match bf89793 timing semantics: start automatically when the ball
         * actually leaves O for +5 cm, not when KEY0 was pressed.
         */
        if (!s_motion_timer_started)
        {
            s_motion_start_ms = now_ms;
            s_motion_timer_started = true;
            (void)BSP_Debug_Printf(
                "BALL_BAL,TIMER,START,AT=O_DEPARTURE\r\n");
        }

        Ball_EnterState(BALL_STATE_TO_POS5, now_ms);
        Ball_SetPulse(SERVO_RIGHT_CRUISE_US, "RIGHT_CRUISE");
    }
}

static void Ball_ControlToPos5(uint32_t now_ms)
{
    int32_t distance;
    int32_t brake_distance;
    int32_t brake_offset;

    if (s_last_cx > s_pos5_peak_cx)
    {
        s_pos5_peak_cx = s_last_cx;
    }

    distance = (int32_t)BALL_TARGET_CX_P5 - s_last_cx;

    /* Arrive within about 7 mm, then immediately command a real reversal. */
    if (distance <= BALL_POS5_SWITCH_ERR_PX)
    {
        s_pos5_reached_ms = Ball_GetMotionElapsedMs(now_ms);
        Ball_EnterState(BALL_STATE_TURN_BRAKE, now_ms);
        Ball_SetPulse(SERVO_TURN_BRAKE_US, "TURN_HARD_BRAKE");
        return;
    }

    brake_distance = BALL_POS5_BRAKE_BASE_PX +
        (Ball_Abs32(s_speed) * BALL_POS5_BRAKE_SPEED_GAIN);

    if ((s_speed > 0) && (distance <= brake_distance))
    {
        brake_offset = 90 + (Ball_Abs32(s_speed) * 5);
        brake_offset = Ball_Clamp32(brake_offset, 90, 250);
        Ball_SetPulseSigned(
            (int32_t)SERVO_NEUTRAL_US - brake_offset,
            "POS5_BRAKE");
    }
    else
    {
        Ball_SetPulse(SERVO_RIGHT_CRUISE_US, "RIGHT_CRUISE");
    }
}

static void Ball_ControlTurnBrake(uint32_t now_ms)
{
    if (s_last_cx > s_pos5_peak_cx)
    {
        s_pos5_peak_cx = s_last_cx;
    }

    Ball_SetPulse(SERVO_TURN_BRAKE_US, "TURN_HARD_BRAKE");

    if (s_speed <= 0)
    {
        s_turn_zero_frames++;
    }
    else
    {
        s_turn_zero_frames = 0U;
    }

    if ((s_turn_zero_frames >= 2U) ||
        ((uint32_t)(now_ms - s_state_start_ms) >= BALL_TURN_BRAKE_MAX_MS))
    {
        Ball_EnterState(BALL_STATE_TO_NEG5, now_ms);
        Ball_SetPulse(SERVO_LEFT_CRUISE_US, "LEFT_CRUISE");
    }
}

static void Ball_ControlToNeg5(uint32_t now_ms)
{
    int32_t distance;
    int32_t brake_distance;
    int32_t brake_offset;

    if (s_last_cx < s_neg5_min_cx)
    {
        s_neg5_min_cx = s_last_cx;
    }

    distance = s_last_cx - (int32_t)BALL_TARGET_CX_N5;

    if ((Ball_Abs32(s_last_error) <= BALL_NEG5_CAPTURE_ERR_PX) ||
        (s_last_cx <= BALL_TARGET_CX_N5))
    {
        Ball_EnterState(BALL_STATE_SETTLE_NEG5, now_ms);
        return;
    }

    /* If the ball still travels right after +5, brake it left immediately. */
    if (s_speed > 2)
    {
        Ball_SetPulse(SERVO_TURN_BRAKE_US, "TURN_HARD_BRAKE");
        return;
    }

    brake_distance = BALL_NEG5_BRAKE_BASE_PX +
        (Ball_Abs32(s_speed) * BALL_NEG5_BRAKE_SPEED_GAIN);

    if ((s_speed < 0) && (distance <= brake_distance))
    {
        brake_offset = 105 + (Ball_Abs32(s_speed) * 5);
        brake_offset = Ball_Clamp32(brake_offset, 105, 270);
        Ball_SetPulseSigned(
            (int32_t)SERVO_NEUTRAL_US + brake_offset,
            "NEG5_PREDICT_BRAKE");
    }
    else
    {
        Ball_SetPulse(SERVO_LEFT_CRUISE_US, "LEFT_CRUISE");
    }
}

static void Ball_ControlFinishedAntiDrift(void)
{
    int32_t motion_speed;
    int32_t position_correction = 0;
    int32_t speed_correction;
    int32_t total_correction;
    int32_t target_pulse;

    /* Use the larger speed estimate so a fresh drift is opposed immediately. */
    motion_speed = (Ball_Abs32(s_raw_speed) > Ball_Abs32(s_speed))
        ? s_raw_speed
        : s_speed;

    if (motion_speed > 1)
    {
        /* cx/x_mm is increasing: tilt left by reducing pulse. */
        s_finish_hold_bias_us -= BALL_FINISH_HOLD_BIAS_STEP_US;
    }
    else if (motion_speed < -1)
    {
        /* cx/x_mm is decreasing: tilt right by increasing pulse. */
        s_finish_hold_bias_us += BALL_FINISH_HOLD_BIAS_STEP_US;
    }
    else if (s_last_error_mm < -BALL_FINISH_HOLD_INNER_MM)
    {
        s_finish_hold_bias_us -= 1;
    }
    else if (s_last_error_mm > BALL_FINISH_HOLD_INNER_MM)
    {
        s_finish_hold_bias_us += 1;
    }

    s_finish_hold_bias_us = Ball_Clamp32(
        s_finish_hold_bias_us,
        -BALL_FINISH_HOLD_BIAS_MAX_US,
        BALL_FINISH_HOLD_BIAS_MAX_US);

    if (Ball_Abs32(s_last_error_mm) > BALL_FINISH_HOLD_INNER_MM)
    {
        position_correction = Ball_Clamp32(
            s_last_error_mm * BALL_FINISH_HOLD_POS_GAIN_US,
            -BALL_FINISH_HOLD_POS_MAX_US,
            BALL_FINISH_HOLD_POS_MAX_US);
    }

    if (Ball_Abs32(motion_speed) <= 1)
    {
        speed_correction = 0;
    }
    else
    {
        speed_correction = Ball_Clamp32(
            -motion_speed * BALL_FINISH_HOLD_SPEED_GAIN_US,
            -BALL_FINISH_HOLD_SPEED_MAX_US,
            BALL_FINISH_HOLD_SPEED_MAX_US);
    }

    total_correction = Ball_Clamp32(
        s_finish_hold_bias_us + position_correction + speed_correction,
        -BALL_FINISH_HOLD_TOTAL_MAX_US,
        BALL_FINISH_HOLD_TOTAL_MAX_US);

    target_pulse = (int32_t)SERVO_NEUTRAL_US + total_correction;
    Ball_SlewPulse(
        target_pulse,
        BALL_FINISH_HOLD_SLEW_US,
        "FINISHED_ANTI_DRIFT");
}

static void Ball_ControlFinalHold(bool allow_finish, uint32_t now_ms)
{
    int32_t error_abs;
    int32_t speed_abs;
    int32_t motion_speed;
    int32_t motion_speed_abs;
    int32_t brake_offset;
    int32_t correction;
    int32_t direction;

    if (s_last_cx < s_neg5_min_cx)
    {
        s_neg5_min_cx = s_last_cx;
    }

    error_abs = Ball_Abs32(s_last_error);
    speed_abs = Ball_Abs32(s_speed);
    motion_speed = (Ball_Abs32(s_raw_speed) > speed_abs)
        ? s_raw_speed
        : s_speed;
    motion_speed_abs = Ball_Abs32(motion_speed);

    /* First remove kinetic energy. Never go neutral while crossing fast. */
    if (motion_speed_abs > BALL_FINAL_SPEED_PX)
    {
        brake_offset = BALL_CAPTURE_BRAKE_BASE_US +
            (motion_speed_abs * BALL_CAPTURE_BRAKE_SPEED_GAIN);
        brake_offset = Ball_Clamp32(
            brake_offset,
            BALL_CAPTURE_BRAKE_BASE_US,
            BALL_CAPTURE_BRAKE_MAX_US);

        if (motion_speed < 0)
        {
            Ball_SetPulseSigned(
                (int32_t)SERVO_NEUTRAL_US + brake_offset,
                "CAPTURE_BRAKE_RIGHT");
        }
        else
        {
            Ball_SetPulseSigned(
                (int32_t)SERVO_NEUTRAL_US - brake_offset,
                "CAPTURE_BRAKE_LEFT");
        }

        s_final_stable_frames = 0U;
        s_settle_stuck_frames = 0U;
        s_settle_kick_frames = 0U;
        return;
    }

    if (Ball_Abs32(s_last_error_mm) <= BALL_FINAL_TOLERANCE_MM)
    {
        /*
         * Competition rule: +/-1 cm is acceptable. The first time the ball
         * has actually stopped inside that window, finish immediately.
         * Require both raw and filtered speed to be small so a single repeated
         * camera coordinate cannot falsely declare a fast crossing as stopped.
         */
        Ball_SetPulse(SERVO_NEUTRAL_US, "FINAL_WINDOW");
        s_settle_stuck_frames = 0U;
        s_settle_kick_frames = 0U;

        if (allow_finish &&
            (speed_abs <= BALL_FINAL_SPEED_PX) &&
            (Ball_Abs32(s_raw_speed) <= BALL_FINAL_SPEED_PX))
        {
            s_final_stable_frames = 1U;
            s_finish_hold_bias_us = Ball_Clamp32(
                s_last_error_mm * 2,
                -30,
                30);
            Ball_EnterState(BALL_STATE_FINISHED, now_ms);
            Ball_SetPulseSigned(
                (int32_t)SERVO_NEUTRAL_US + s_finish_hold_bias_us,
                "FINAL_HOLD_BIAS");
            Ball_ReportResult(true, "FIRST_STOP_WITH_ANTI_DRIFT_HOLD", now_ms);
        }
        return;
    }

    s_final_stable_frames = 0U;
    direction = (s_last_error > 0) ? 1 : -1;

    if (s_settle_kick_frames > 0U)
    {
        s_settle_kick_frames--;
        Ball_SetPulseSigned(
            (int32_t)SERVO_NEUTRAL_US +
                (s_settle_kick_dir * BALL_SETTLE_KICK_US),
            "SETTLE_KICK");
        return;
    }

    correction = (error_abs * 3) / 2;
    correction = Ball_Clamp32(
        correction,
        BALL_SETTLE_CORR_MIN_US,
        BALL_SETTLE_CORR_MAX_US);

    Ball_SetPulseSigned(
        (int32_t)SERVO_NEUTRAL_US + (direction * correction),
        "SETTLE_CORRECT");

    if (speed_abs <= 1)
    {
        s_settle_stuck_frames++;
        if (s_settle_stuck_frames >= BALL_SETTLE_STUCK_WAIT)
        {
            s_settle_stuck_frames = 0U;
            s_settle_kick_dir = direction;
            s_settle_kick_frames = BALL_SETTLE_KICK_FRAMES;
            Ball_SetPulseSigned(
                (int32_t)SERVO_NEUTRAL_US +
                    (direction * BALL_SETTLE_KICK_US),
                "SETTLE_KICK");
        }
    }
    else
    {
        s_settle_stuck_frames = 0U;
    }
}

bool Test_BallBalance_Init(void)
{
    uint32_t now_ms;

    s_initialized = false;
    s_vision_ok = false;
    s_result_done = false;
    s_result_passed = false;
    s_state = BALL_STATE_WAIT_CENTER;
    s_target_cx = BALL_TARGET_CX_O;
    s_servo_pulse = SERVO_NEUTRAL_US;
    s_control_mode = "HOLD";

    s_last_vision_sequence = 0U;
    s_lost_frames = 0U;
    s_last_cx = 0;
    s_last_x_mm = 0;
    s_last_error_mm = 0;
    s_prev_cx = 0;
    s_raw_speed = 0;
    s_speed = 0;
    s_last_error = 0;
    s_has_target = false;

    s_center_ready_frames = 0U;
    s_turn_zero_frames = 0U;
    Ball_ResetSettleCounters();
    s_pos5_peak_cx = BALL_TARGET_CX_O;
    s_neg5_min_cx = BALL_TARGET_CX_O;
    s_pos5_reached_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_motion_start_ms = 0U;
    s_motion_timer_started = false;
    s_finish_hold_bias_us = 0;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_Servo_Init())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_INIT\r\n");
        return false;
    }

    if (!BSP_Servo_SetPulseUs(SERVO_NEUTRAL_US))
    {
        (void)BSP_Debug_Printf("ERR,SERVO_SET_NEUTRAL\r\n");
        return false;
    }

    if (!BSP_Servo_Enable())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_ENABLE\r\n");
        return false;
    }

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
    s_task_start_ms = now_ms;
    s_state_start_ms = now_ms;
    s_last_print_ms = now_ms;
    s_last_heartbeat_ms = now_ms;
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "BALL_BAL,START,CTRL=ONE_SHOT_PREDICT_CAPTURE,"
        "SEQ=O_TO_POS5_TO_NEG5,LIMIT_MS=%lu,"
        "O=%u,POS5=%u,NEG5=%u,NEUTRAL=%u,"
        "POS5_TOL=%d,NEG5_CAPTURE=%d,FINAL_TOL_MM=%d,FINAL_SPD=%d,"
        "FINISH=FIRST_STOP_IN_WINDOW\r\n",
        (unsigned long)BALL_RUN_LIMIT_MS,
        (unsigned int)BALL_TARGET_CX_O,
        (unsigned int)BALL_TARGET_CX_P5,
        (unsigned int)BALL_TARGET_CX_N5,
        (unsigned int)SERVO_NEUTRAL_US,
        BALL_POS5_SWITCH_ERR_PX,
        BALL_NEG5_CAPTURE_ERR_PX,
        BALL_FINAL_TOLERANCE_MM,
        BALL_FINAL_SPEED_PX);

    return true;
}

void Test_BallBalance_Update(void)
{
    uint32_t now_ms;
    VisionStatus_t vision_status;
    bool have_status;
    bool new_valid_frame = false;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    if (s_vision_ok)
    {
        Vision_Update();
    }

    have_status = s_vision_ok && Vision_GetStatus(&vision_status);
    now_ms = HAL_GetTick();

    if (have_status &&
        vision_status.has_frame &&
        (vision_status.sequence != s_last_vision_sequence))
    {
        s_last_vision_sequence = vision_status.sequence;

        if (vision_status.frame.found)
        {
            s_last_cx = (int32_t)vision_status.frame.center_x;
            s_last_x_mm = (int32_t)vision_status.frame.physical_x_mm;
            s_last_error_mm = BALL_TARGET_N5_MM - s_last_x_mm;
            s_last_error = (int32_t)s_target_cx - s_last_cx;
            s_has_target = true;
            s_lost_frames = 0U;

            if (s_prev_cx != 0)
            {
                s_raw_speed = s_last_cx - s_prev_cx;
                /* 1/2 low-pass: noise reduction without the old long phase lag. */
                s_speed = (s_speed + s_raw_speed) / 2;
            }
            else
            {
                s_raw_speed = 0;
                s_speed = 0;
            }
            s_prev_cx = s_last_cx;
            new_valid_frame = true;
        }
        else
        {
            s_has_target = false;
            s_lost_frames++;
        }
    }

    if (new_valid_frame)
    {
        switch (s_state)
        {
            case BALL_STATE_WAIT_CENTER:
                Ball_ControlWaitCenter(now_ms);
                break;

            case BALL_STATE_TO_POS5:
                Ball_ControlToPos5(now_ms);
                break;

            case BALL_STATE_TURN_BRAKE:
                Ball_ControlTurnBrake(now_ms);
                break;

            case BALL_STATE_TO_NEG5:
                Ball_ControlToNeg5(now_ms);
                break;

            case BALL_STATE_SETTLE_NEG5:
                Ball_ControlFinalHold(true, now_ms);
                break;

            case BALL_STATE_FINISHED:
                /*
                 * Result/time stay latched, but keep a gentle background loop
                 * so the ball cannot roll out of the accepted +/-1 cm window.
                 */
                Ball_ControlFinishedAntiDrift();
                break;

            case BALL_STATE_TIMEOUT:
            default:
                Ball_SetPulse(SERVO_NEUTRAL_US, "TIMEOUT_HOLD");
                break;
        }
    }

    if ((s_lost_frames >= BALL_LOST_TIMEOUT_FRAMES) &&
        (s_state != BALL_STATE_FINISHED))
    {
        Ball_SetPulse(SERVO_NEUTRAL_US, "VISION_LOST");
        s_lost_frames = 0U;
    }

    if ((!s_result_done) &&
        s_motion_timer_started &&
        (Ball_GetMotionElapsedMs(now_ms) >= BALL_RUN_LIMIT_MS))
    {
        s_state = BALL_STATE_TIMEOUT;
        s_target_cx = BALL_TARGET_CX_N5;
        s_last_error = (int32_t)s_target_cx - s_last_cx;
        Ball_SetPulse(SERVO_NEUTRAL_US, "TIMEOUT_HOLD");
        Ball_ReportResult(false, "RUN_LIMIT", now_ms);
    }

    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >= BALL_HEARTBEAT_MS)
    {
        s_last_heartbeat_ms = now_ms;
        (void)BSP_Debug_Printf(
            "BALL_BAL,HB,STATE=%s,T=%lums,CX=%ld,ERR=%+ld,SPD=%+ld,"
            "PULSE=%u,VISION=%u\r\n",
            Ball_StateName(s_state),
            (unsigned long)(now_ms - s_task_start_ms),
            (long)s_last_cx,
            (long)s_last_error,
            (long)s_speed,
            (unsigned int)s_servo_pulse,
            s_vision_ok ? 1U : 0U);
    }

    if ((uint32_t)(now_ms - s_last_print_ms) >= BALL_PRINT_PERIOD_MS)
    {
        s_last_print_ms = now_ms;

        if (s_has_target)
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,STATE=%s,T=%lums,CX=%ld,X_MM=%ld,TARGET=%u,ERR=%+ld,"
                "RAW_SPD=%+ld,SPD=%+ld,MODE=%s,PULSE=%u,STABLE=%lu\r\n",
                Ball_StateName(s_state),
                (unsigned long)(now_ms - s_task_start_ms),
                (long)s_last_cx,
                (long)s_last_x_mm,
                (unsigned int)s_target_cx,
                (long)s_last_error,
                (long)s_raw_speed,
                (long)s_speed,
                s_control_mode,
                (unsigned int)s_servo_pulse,
                (unsigned long)s_final_stable_frames,
                (long)s_finish_hold_bias_us);
        }
        else
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,STATE=%s,T=%lums,NO_TARGET,LOST=%lu,PULSE=%u\r\n",
                Ball_StateName(s_state),
                (unsigned long)(now_ms - s_task_start_ms),
                (unsigned long)s_lost_frames,
                (unsigned int)s_servo_pulse);
        }
    }
}

void Test_BallBalance_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    (void)BSP_Servo_SetPulseUs(SERVO_NEUTRAL_US);
    Vision_Stop();
    BSP_Servo_Disable();
    s_initialized = false;
    s_vision_ok = false;

    (void)BSP_Debug_Printf(
        "BALL_BAL,STOP,RESULT=%s,PULSE=%u\r\n",
        s_result_passed ? "PASS" : (s_result_done ? "FAIL" : "ABORT"),
        (unsigned int)SERVO_NEUTRAL_US);
}

bool Test_BallBalance_IsInitialized(void)
{
    return s_initialized;
}

bool Test_BallBalance_IsFinished(void)
{
    return s_result_done;
}

bool Test_BallBalance_Passed(void)
{
    return s_result_done && s_result_passed;
}

bool Test_BallBalance_IsTimerRunning(void)
{
    return s_initialized &&
           s_motion_timer_started &&
           (!s_result_done);
}

uint32_t Test_BallBalance_GetElapsedMs(void)
{
    if (s_result_done)
    {
        return s_result_elapsed_ms;
    }

    if (s_initialized && s_motion_timer_started)
    {
        return Ball_GetMotionElapsedMs(HAL_GetTick());
    }

    return 0U;
}
