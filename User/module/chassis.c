#include "chassis.h"

#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "chassis_config.h"
#include "chassis_speed_profile.h"
#include "stm32f4xx_hal.h"
#include "wheel_speed_control.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static bool s_enabled = false;
static bool s_emergency_latched = false;

static WheelSpeedController_t s_left_controller;
static WheelSpeedController_t s_right_controller;
static ChassisSpeedProfile_t s_speed_profile;
static ChassisSpeedProfileStatus_t s_profile_status;

static ChassisStatus_t s_status;
static uint32_t s_last_control_ms = 0U;
static uint16_t s_stopped_stable_ms = 0U;

/* 巡线专用双通道状态：基础速度走S曲线，转向修正快速限速率。 */
static bool s_line_follow_active = false;
static int32_t s_line_follow_turn_command_mm_s = 0;
static int32_t s_line_follow_turn_ramped_mm_s = 0;
static int32_t s_line_follow_turn_applied_mm_s = 0;

static int32_t Chassis_DivideRounded(int64_t numerator,
                                     int64_t denominator)
{
    if (denominator <= 0)
    {
        return 0;
    }

    if (numerator >= 0)
    {
        return (int32_t)(
            (numerator + (denominator / 2LL)) / denominator);
    }

    return (int32_t)(
        (numerator - (denominator / 2LL)) / denominator);
}

static int32_t Chassis_ClampMmps(int64_t speed_mm_s)
{
    if (speed_mm_s > (int64_t)CHASSIS_MAX_WHEEL_SPEED_MM_S)
    {
        return CHASSIS_MAX_WHEEL_SPEED_MM_S;
    }

    if (speed_mm_s < -(int64_t)CHASSIS_MAX_WHEEL_SPEED_MM_S)
    {
        return -CHASSIS_MAX_WHEEL_SPEED_MM_S;
    }

    return (int32_t)speed_mm_s;
}

static int32_t Chassis_ClampCps(int64_t speed_cps)
{
    if (speed_cps > (int64_t)CHASSIS_MAX_WHEEL_SPEED_CPS)
    {
        return CHASSIS_MAX_WHEEL_SPEED_CPS;
    }

    if (speed_cps < -(int64_t)CHASSIS_MAX_WHEEL_SPEED_CPS)
    {
        return -CHASSIS_MAX_WHEEL_SPEED_CPS;
    }

    return (int32_t)speed_cps;
}

static int32_t Chassis_ClampRangeI32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
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

static int32_t Chassis_ApproachI32(
    int32_t current,
    int32_t target,
    int32_t step)
{
    int64_t difference;

    if (step <= 0)
    {
        return target;
    }

    difference = (int64_t)target - (int64_t)current;
    if (difference > (int64_t)step)
    {
        return current + step;
    }

    if (difference < -(int64_t)step)
    {
        return current - step;
    }

    return target;
}

static int32_t Chassis_RateStepMmps(
    int32_t rate_mm_s2,
    uint16_t dt_ms)
{
    int64_t step;

    if ((rate_mm_s2 <= 0) || (dt_ms == 0U))
    {
        return 0;
    }

    step = ((int64_t)rate_mm_s2 * (int64_t)dt_ms + 999LL) /
           1000LL;
    if (step < 1LL)
    {
        step = 1LL;
    }
    if (step > INT32_MAX)
    {
        return INT32_MAX;
    }

    return (int32_t)step;
}

static int32_t Chassis_AbsI32(int32_t value)
{
    if (value >= 0)
    {
        return value;
    }

    if (value == INT32_MIN)
    {
        return INT32_MAX;
    }

    return -value;
}

static int32_t Chassis_AddSaturated(int32_t total, int16_t delta)
{
    int64_t result = (int64_t)total + (int64_t)delta;

    if (result > INT32_MAX)
    {
        return INT32_MAX;
    }

    if (result < INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)result;
}

static int32_t Chassis_AbsDelta(int16_t delta)
{
    return (delta < 0) ? -(int32_t)delta : (int32_t)delta;
}

static int32_t Chassis_GetAllowedEncoderDelta(uint16_t dt_ms)
{
    int64_t scaled_delta;

    scaled_delta =
        ((int64_t)CHASSIS_ENCODER_PLAUSIBLE_MAX_CPS *
         (int64_t)dt_ms +
         999LL) /
        1000LL;

    scaled_delta +=
        (int64_t)CHASSIS_ENCODER_DELTA_MARGIN_COUNTS;

    if (scaled_delta > INT16_MAX)
    {
        return INT16_MAX;
    }

    return (int32_t)scaled_delta;
}

static void Chassis_ClearControlStatus(void)
{
    s_status.left_feedforward_pwm = 0;
    s_status.right_feedforward_pwm = 0;
    s_status.left_proportional_pwm = 0;
    s_status.right_proportional_pwm = 0;
    s_status.left_integral_pwm = 0;
    s_status.right_integral_pwm = 0;
    s_status.left_pwm = 0;
    s_status.right_pwm = 0;
    s_status.left_output_saturated = false;
    s_status.right_output_saturated = false;
}

int32_t Chassis_MmpsToCps(int32_t speed_mm_s)
{
    return Chassis_DivideRounded(
        (int64_t)speed_mm_s *
        (int64_t)BSP_ENCODER_COUNTS_PER_REV *
        1000LL,
        (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM);
}

int32_t Chassis_CpsToMmps(int32_t speed_cps)
{
    return Chassis_DivideRounded(
        (int64_t)speed_cps *
        (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM,
        (int64_t)BSP_ENCODER_COUNTS_PER_REV *
        1000LL);
}

static ChassisMotionMode_t Chassis_MapProfileMode(
    ChassisSpeedProfileMode_t mode)
{
    switch (mode)
    {
        case CHASSIS_SPEED_PROFILE_MODE_DRIVE:
            return CHASSIS_MOTION_MODE_DRIVE;
        case CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP:
            return CHASSIS_MOTION_MODE_SOFT_STOP;
        case CHASSIS_SPEED_PROFILE_MODE_FAST_STOP:
            return CHASSIS_MOTION_MODE_FAST_STOP;
        case CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP:
            return CHASSIS_MOTION_MODE_REVERSAL_STOP;
        case CHASSIS_SPEED_PROFILE_MODE_IDLE:
        default:
            return CHASSIS_MOTION_MODE_IDLE;
    }
}

static void Chassis_ResetSpeedProfile(void)
{
    ChassisSpeedProfile_Reset(&s_speed_profile);
    memset(&s_profile_status, 0, sizeof(s_profile_status));

    s_stopped_stable_ms = 0U;
    s_status.speed_ramp_active = false;
    s_status.motion_stopped = true;
    s_status.motion_mode = CHASSIS_MOTION_MODE_IDLE;
    s_status.stopped_stable_ms = 0U;

    s_status.left_command_mm_s = 0;
    s_status.right_command_mm_s = 0;
    s_status.left_command_cps = 0;
    s_status.right_command_cps = 0;
    s_status.left_target_mm_s = 0;
    s_status.right_target_mm_s = 0;
    s_status.left_target_cps = 0;
    s_status.right_target_cps = 0;

    s_status.forward_command_mm_s = 0;
    s_status.turn_command_mm_s = 0;
    s_status.forward_target_mm_s = 0;
    s_status.turn_target_mm_s = 0;
    s_status.forward_accel_mm_s2 = 0;
    s_status.turn_accel_mm_s2 = 0;
    s_status.stop_reference_mm_s = 0;
    s_status.stop_accel_mm_s2 = 0;

    s_line_follow_active = false;
    s_line_follow_turn_command_mm_s = 0;
    s_line_follow_turn_ramped_mm_s = 0;
    s_line_follow_turn_applied_mm_s = 0;
    s_status.line_follow_active = false;
    s_status.line_follow_turn_command_mm_s = 0;
    s_status.line_follow_turn_ramped_mm_s = 0;
}

static bool Chassis_UpdateSpeedProfile(uint16_t dt_ms)
{
    int32_t left_target_mm_s;
    int32_t right_target_mm_s;
    int32_t left_target_cps;
    int32_t right_target_cps;

    if (!ChassisSpeedProfile_Update(&s_speed_profile, dt_ms))
    {
        return false;
    }

    if (!ChassisSpeedProfile_GetStatus(
            &s_speed_profile,
            &s_profile_status))
    {
        return false;
    }

    left_target_mm_s = s_profile_status.output_left_mm_s;
    right_target_mm_s = s_profile_status.output_right_mm_s;

    if (s_line_follow_active &&
        (s_profile_status.mode == CHASSIS_SPEED_PROFILE_MODE_DRIVE))
    {
        int32_t base_target_mm_s;
        int32_t turn_step_mm_s;
        int32_t safe_turn_limit_mm_s;
        int32_t speed_headroom_mm_s;
        int32_t previous_turn_mm_s;
        int32_t turn_accel_mm_s2;

        base_target_mm_s =
            (s_profile_status.output_left_mm_s +
             s_profile_status.output_right_mm_s) /
            2;
        base_target_mm_s = Chassis_ClampRangeI32(
            base_target_mm_s,
            0,
            CHASSIS_LINE_FOLLOW_MAX_WHEEL_SPEED_MM_S);

        turn_step_mm_s = Chassis_RateStepMmps(
            CHASSIS_LINE_FOLLOW_TURN_SLEW_MM_S2,
            dt_ms);
        s_line_follow_turn_ramped_mm_s = Chassis_ApproachI32(
            s_line_follow_turn_ramped_mm_s,
            s_line_follow_turn_command_mm_s,
            turn_step_mm_s);

        speed_headroom_mm_s =
            CHASSIS_LINE_FOLLOW_MAX_WHEEL_SPEED_MM_S -
            base_target_mm_s;
        safe_turn_limit_mm_s =
            (base_target_mm_s < speed_headroom_mm_s) ?
            base_target_mm_s : speed_headroom_mm_s;
        if (safe_turn_limit_mm_s < 0)
        {
            safe_turn_limit_mm_s = 0;
        }

        previous_turn_mm_s = s_line_follow_turn_applied_mm_s;
        s_line_follow_turn_applied_mm_s = Chassis_ClampRangeI32(
            s_line_follow_turn_ramped_mm_s,
            -safe_turn_limit_mm_s,
            safe_turn_limit_mm_s);

        left_target_mm_s = base_target_mm_s +
                           s_line_follow_turn_applied_mm_s;
        right_target_mm_s = base_target_mm_s -
                            s_line_follow_turn_applied_mm_s;

        turn_accel_mm_s2 = Chassis_DivideRounded(
            (int64_t)(s_line_follow_turn_applied_mm_s -
                      previous_turn_mm_s) *
            1000LL,
            (int64_t)dt_ms);

        s_status.forward_target_mm_s = base_target_mm_s;
        s_status.turn_target_mm_s =
            s_line_follow_turn_applied_mm_s;
        s_status.forward_accel_mm_s2 =
            s_profile_status.forward_accel_mm_s2;
        s_status.turn_accel_mm_s2 = turn_accel_mm_s2;
        s_status.stop_reference_mm_s = 0;
        s_status.stop_accel_mm_s2 = 0;
        s_status.speed_ramp_active =
            s_profile_status.active ||
            (s_line_follow_turn_ramped_mm_s !=
             s_line_follow_turn_command_mm_s) ||
            (s_line_follow_turn_applied_mm_s !=
             s_line_follow_turn_ramped_mm_s);
        s_status.motion_mode = CHASSIS_MOTION_MODE_DRIVE;
    }
    else
    {
        s_status.forward_target_mm_s =
            s_profile_status.forward_speed_mm_s;
        s_status.turn_target_mm_s =
            s_profile_status.turn_speed_mm_s;
        s_status.forward_accel_mm_s2 =
            s_profile_status.forward_accel_mm_s2;
        s_status.turn_accel_mm_s2 =
            s_profile_status.turn_accel_mm_s2;
        s_status.stop_reference_mm_s =
            s_profile_status.stop_reference_mm_s;
        s_status.stop_accel_mm_s2 =
            s_profile_status.stop_accel_mm_s2;
        s_status.speed_ramp_active = s_profile_status.active;
        s_status.motion_mode =
            Chassis_MapProfileMode(s_profile_status.mode);
    }

    left_target_cps = Chassis_ClampCps(
        Chassis_MmpsToCps(left_target_mm_s));
    right_target_cps = Chassis_ClampCps(
        Chassis_MmpsToCps(right_target_mm_s));

    if (!WheelSpeedControl_SetTargetCps(
            &s_left_controller,
            left_target_cps))
    {
        return false;
    }

    if (!WheelSpeedControl_SetTargetCps(
            &s_right_controller,
            right_target_cps))
    {
        return false;
    }

    s_status.left_target_mm_s = left_target_mm_s;
    s_status.right_target_mm_s = right_target_mm_s;
    s_status.left_target_cps = left_target_cps;
    s_status.right_target_cps = right_target_cps;
    s_status.line_follow_active = s_line_follow_active;
    s_status.line_follow_turn_command_mm_s =
        s_line_follow_turn_command_mm_s;
    s_status.line_follow_turn_ramped_mm_s =
        s_line_follow_turn_applied_mm_s;

    return true;
}

static void Chassis_UpdateStoppedState(uint16_t dt_ms)
{
    bool measured_slow;
    bool profile_zero;
    uint32_t next_hold;

    measured_slow =
        (Chassis_AbsI32(s_status.left_measured_mm_s) <=
         CHASSIS_STOP_MEASURED_THRESHOLD_MM_S) &&
        (Chassis_AbsI32(s_status.right_measured_mm_s) <=
         CHASSIS_STOP_MEASURED_THRESHOLD_MM_S);

    profile_zero =
        s_profile_status.stopped &&
        (s_status.left_target_cps == 0) &&
        (s_status.right_target_cps == 0);

    if (profile_zero && measured_slow)
    {
        next_hold = (uint32_t)s_stopped_stable_ms +
                    (uint32_t)dt_ms;

        if (next_hold > (uint32_t)CHASSIS_STOP_STABLE_HOLD_MS)
        {
            next_hold = CHASSIS_STOP_STABLE_HOLD_MS;
        }

        s_stopped_stable_ms = (uint16_t)next_hold;
    }
    else
    {
        s_stopped_stable_ms = 0U;
    }

    s_status.stopped_stable_ms = s_stopped_stable_ms;
    s_status.motion_stopped =
        s_stopped_stable_ms >= CHASSIS_STOP_STABLE_HOLD_MS;
}

bool Chassis_Init(void)
{
    WheelSpeedConfig_t left_config;
    WheelSpeedConfig_t right_config;
    ChassisSpeedProfileConfig_t profile_config;

    s_initialized = false;
    s_enabled = false;
    s_emergency_latched = false;
    memset(&s_status, 0, sizeof(s_status));
    memset(&s_speed_profile, 0, sizeof(s_speed_profile));
    memset(&s_profile_status, 0, sizeof(s_profile_status));

    if (!BSP_Motor_Init())
    {
        return false;
    }

    if (!BSP_Encoder_Init())
    {
        return false;
    }

    left_config.kp_q10 = WHEEL_SPEED_LEFT_KP_Q10;
    left_config.ki_q10 = WHEEL_SPEED_LEFT_KI_Q10;
    left_config.feedforward_gain_q10 =
        WHEEL_SPEED_LEFT_FF_GAIN_Q10;
    left_config.feedforward_static_pwm =
        WHEEL_SPEED_LEFT_FF_STATIC_PWM;
    left_config.pwm_limit = CHASSIS_PWM_LIMIT;
    left_config.integral_limit_pwm =
        WHEEL_SPEED_INTEGRAL_LIMIT_PWM;
    left_config.min_drive_pwm =
        WHEEL_SPEED_LEFT_MIN_DRIVE_PWM;
    left_config.nominal_period_ms =
        CHASSIS_CONTROL_PERIOD_MS;
    left_config.measurement_window =
        WHEEL_SPEED_MEASUREMENT_WINDOW;

    right_config.kp_q10 = WHEEL_SPEED_RIGHT_KP_Q10;
    right_config.ki_q10 = WHEEL_SPEED_RIGHT_KI_Q10;
    right_config.feedforward_gain_q10 =
        WHEEL_SPEED_RIGHT_FF_GAIN_Q10;
    right_config.feedforward_static_pwm =
        WHEEL_SPEED_RIGHT_FF_STATIC_PWM;
    right_config.pwm_limit = CHASSIS_PWM_LIMIT;
    right_config.integral_limit_pwm =
        WHEEL_SPEED_INTEGRAL_LIMIT_PWM;
    right_config.min_drive_pwm =
        WHEEL_SPEED_RIGHT_MIN_DRIVE_PWM;
    right_config.nominal_period_ms =
        CHASSIS_CONTROL_PERIOD_MS;
    right_config.measurement_window =
        WHEEL_SPEED_MEASUREMENT_WINDOW;

    if (!WheelSpeedControl_Init(
            &s_left_controller,
            &left_config))
    {
        return false;
    }

    if (!WheelSpeedControl_Init(
            &s_right_controller,
            &right_config))
    {
        return false;
    }

    profile_config.max_wheel_speed_mm_s =
        CHASSIS_MAX_WHEEL_SPEED_MM_S;
    profile_config.forward_accel_mm_s2 =
        CHASSIS_PROFILE_FORWARD_ACCEL_MM_S2;
    profile_config.forward_decel_mm_s2 =
        CHASSIS_PROFILE_FORWARD_DECEL_MM_S2;
    profile_config.forward_accel_jerk_mm_s3 =
        CHASSIS_PROFILE_FORWARD_ACCEL_JERK_MM_S3;
    profile_config.forward_decel_jerk_mm_s3 =
        CHASSIS_PROFILE_FORWARD_DECEL_JERK_MM_S3;
    profile_config.turn_accel_mm_s2 =
        CHASSIS_PROFILE_TURN_ACCEL_MM_S2;
    profile_config.turn_jerk_mm_s3 =
        CHASSIS_PROFILE_TURN_JERK_MM_S3;
    profile_config.soft_stop_decel_mm_s2 =
        CHASSIS_PROFILE_SOFT_STOP_DECEL_MM_S2;
    profile_config.soft_stop_jerk_mm_s3 =
        CHASSIS_PROFILE_SOFT_STOP_JERK_MM_S3;
    profile_config.fast_stop_decel_mm_s2 =
        CHASSIS_PROFILE_FAST_STOP_DECEL_MM_S2;
    profile_config.fast_stop_jerk_mm_s3 =
        CHASSIS_PROFILE_FAST_STOP_JERK_MM_S3;
    profile_config.speed_snap_mm_s =
        CHASSIS_PROFILE_SPEED_SNAP_MM_S;
    profile_config.accel_snap_mm_s2 =
        CHASSIS_PROFILE_ACCEL_SNAP_MM_S2;

    if (!ChassisSpeedProfile_Init(
            &s_speed_profile,
            &profile_config))
    {
        return false;
    }

    Chassis_ResetSpeedProfile();
    s_last_control_ms = HAL_GetTick();
    s_initialized = true;
    s_status.initialized = true;
    s_status.motion_stopped = true;

    return true;
}

bool Chassis_Enable(bool enable)
{
    if (!s_initialized)
    {
        return false;
    }

    if (enable)
    {
        BSP_Encoder_Reset();

        memset(&s_status, 0, sizeof(s_status));
        s_status.initialized = true;
        s_status.encoder_sample_valid = true;
        s_status.motion_stopped = true;

        Chassis_ResetSpeedProfile();
        WheelSpeedControl_Reset(&s_left_controller);
        WheelSpeedControl_Reset(&s_right_controller);

        if (!BSP_Motor_Enable(true))
        {
            return false;
        }

        s_emergency_latched = false;
        s_status.emergency_latched = false;
        s_last_control_ms = HAL_GetTick();
        s_enabled = true;
        s_status.enabled = true;
        return true;
    }

    Chassis_Stop();
    (void)BSP_Motor_Enable(false);

    s_enabled = false;
    s_status.enabled = false;
    return true;
}

bool Chassis_IsInitialized(void)
{
    return s_initialized;
}

bool Chassis_IsEnabled(void)
{
    return s_initialized && s_enabled;
}

static bool Chassis_LeaveLineFollowMode(void)
{
    if (!s_line_follow_active)
    {
        return true;
    }

    if (!ChassisSpeedProfile_SynchronizeOutputMmps(
            &s_speed_profile,
            s_status.left_target_mm_s,
            s_status.right_target_mm_s))
    {
        return false;
    }

    s_line_follow_active = false;
    s_line_follow_turn_command_mm_s = 0;
    s_line_follow_turn_ramped_mm_s = 0;
    s_line_follow_turn_applied_mm_s = 0;
    s_status.line_follow_active = false;
    s_status.line_follow_turn_command_mm_s = 0;
    s_status.line_follow_turn_ramped_mm_s = 0;
    return true;
}

bool Chassis_SetWheelSpeedMmps(int32_t left_mm_s,
                               int32_t right_mm_s)
{
    int32_t clamped_left_mm_s;
    int32_t clamped_right_mm_s;

    if ((!s_initialized) || s_emergency_latched)
    {
        return false;
    }

    if (!Chassis_LeaveLineFollowMode())
    {
        return false;
    }

    clamped_left_mm_s =
        Chassis_ClampMmps((int64_t)left_mm_s);
    clamped_right_mm_s =
        Chassis_ClampMmps((int64_t)right_mm_s);

    if (!ChassisSpeedProfile_SetCommandMmps(
            &s_speed_profile,
            clamped_left_mm_s,
            clamped_right_mm_s))
    {
        return false;
    }

    s_status.left_command_mm_s = clamped_left_mm_s;
    s_status.right_command_mm_s = clamped_right_mm_s;
    s_status.left_command_cps =
        Chassis_MmpsToCps(clamped_left_mm_s);
    s_status.right_command_cps =
        Chassis_MmpsToCps(clamped_right_mm_s);
    s_status.forward_command_mm_s =
        (clamped_left_mm_s + clamped_right_mm_s) / 2;
    s_status.turn_command_mm_s =
        (clamped_left_mm_s - clamped_right_mm_s) / 2;

    if ((clamped_left_mm_s != 0) ||
        (clamped_right_mm_s != 0))
    {
        s_status.motion_stopped = false;
        s_stopped_stable_ms = 0U;
    }

    return true;
}

bool Chassis_SetLineFollowCommandMmps(int32_t base_mm_s,
                                      int32_t turn_mm_s)
{
    int32_t max_turn_mm_s;
    int32_t current_left_mm_s;
    int32_t current_right_mm_s;
    int32_t left_command_mm_s;
    int32_t right_command_mm_s;

    if ((!s_initialized) || s_emergency_latched)
    {
        return false;
    }

    base_mm_s = Chassis_ClampRangeI32(
        base_mm_s,
        0,
        CHASSIS_LINE_FOLLOW_MAX_WHEEL_SPEED_MM_S);

    max_turn_mm_s =
        CHASSIS_LINE_FOLLOW_MAX_WHEEL_SPEED_MM_S - base_mm_s;
    if (base_mm_s < max_turn_mm_s)
    {
        max_turn_mm_s = base_mm_s;
    }
    if (max_turn_mm_s < 0)
    {
        max_turn_mm_s = 0;
    }

    turn_mm_s = Chassis_ClampRangeI32(
        turn_mm_s,
        -max_turn_mm_s,
        max_turn_mm_s);

    if (!s_line_follow_active)
    {
        current_left_mm_s = s_status.left_target_mm_s;
        current_right_mm_s = s_status.right_target_mm_s;

        if (!ChassisSpeedProfile_SynchronizeOutputMmps(
                &s_speed_profile,
                current_left_mm_s,
                current_right_mm_s))
        {
            return false;
        }

        s_line_follow_turn_ramped_mm_s =
            (current_left_mm_s - current_right_mm_s) / 2;
        s_line_follow_turn_applied_mm_s =
            s_line_follow_turn_ramped_mm_s;
        s_line_follow_active = true;
    }

    if (!ChassisSpeedProfile_SetCommandMmps(
            &s_speed_profile,
            base_mm_s,
            base_mm_s))
    {
        return false;
    }

    s_line_follow_turn_command_mm_s = turn_mm_s;
    left_command_mm_s = base_mm_s + turn_mm_s;
    right_command_mm_s = base_mm_s - turn_mm_s;

    s_status.left_command_mm_s = left_command_mm_s;
    s_status.right_command_mm_s = right_command_mm_s;
    s_status.left_command_cps =
        Chassis_MmpsToCps(left_command_mm_s);
    s_status.right_command_cps =
        Chassis_MmpsToCps(right_command_mm_s);
    s_status.forward_command_mm_s = base_mm_s;
    s_status.turn_command_mm_s = turn_mm_s;
    s_status.line_follow_active = true;
    s_status.line_follow_turn_command_mm_s = turn_mm_s;

    if ((base_mm_s != 0) || (turn_mm_s != 0))
    {
        s_status.motion_stopped = false;
        s_stopped_stable_ms = 0U;
    }

    return true;
}

bool Chassis_SetWheelSpeedCps(int32_t left_cps,
                              int32_t right_cps)
{
    if (!s_initialized)
    {
        return false;
    }

    left_cps = Chassis_ClampCps((int64_t)left_cps);
    right_cps = Chassis_ClampCps((int64_t)right_cps);

    return Chassis_SetWheelSpeedMmps(
        Chassis_CpsToMmps(left_cps),
        Chassis_CpsToMmps(right_cps));
}

bool Chassis_SetVelocity(int32_t linear_mm_s,
                         int32_t angular_mrad_s)
{
    int64_t turning_mm_s;
    int64_t left_mm_s;
    int64_t right_mm_s;

    if (!s_initialized)
    {
        return false;
    }

    turning_mm_s =
        ((int64_t)angular_mrad_s *
         (int64_t)CHASSIS_TRACK_WIDTH_MM) /
        2000LL;

    left_mm_s = (int64_t)linear_mm_s - turning_mm_s;
    right_mm_s = (int64_t)linear_mm_s + turning_mm_s;

    return Chassis_SetWheelSpeedMmps(
        Chassis_ClampMmps(left_mm_s),
        Chassis_ClampMmps(right_mm_s));
}

bool Chassis_RequestStop(ChassisStopMode_t mode)
{
    ChassisSpeedProfileMode_t profile_mode;

    if (!s_initialized)
    {
        return false;
    }

    if (mode == CHASSIS_STOP_MODE_EMERGENCY)
    {
        Chassis_Stop();
        return true;
    }

    if (s_emergency_latched)
    {
        return false;
    }

    if (!Chassis_LeaveLineFollowMode())
    {
        return false;
    }

    profile_mode =
        (mode == CHASSIS_STOP_MODE_FAST) ?
        CHASSIS_SPEED_PROFILE_MODE_FAST_STOP :
        CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP;

    if (!ChassisSpeedProfile_RequestStop(
            &s_speed_profile,
            profile_mode))
    {
        return false;
    }

    s_status.left_command_mm_s = 0;
    s_status.right_command_mm_s = 0;
    s_status.left_command_cps = 0;
    s_status.right_command_cps = 0;
    s_status.forward_command_mm_s = 0;
    s_status.turn_command_mm_s = 0;
    s_status.motion_stopped = false;
    s_stopped_stable_ms = 0U;
    return true;
}

bool Chassis_IsMotionStopped(void)
{
    if (!s_initialized)
    {
        return true;
    }

    return s_status.motion_stopped || s_emergency_latched;
}

bool Chassis_SetWheelPiGainsQ10(int32_t left_kp_q10,
                                int32_t left_ki_q10,
                                int32_t right_kp_q10,
                                int32_t right_ki_q10)
{
    if (!s_initialized)
    {
        return false;
    }

    if (!WheelSpeedControl_SetGainsQ10(
            &s_left_controller,
            left_kp_q10,
            left_ki_q10))
    {
        return false;
    }

    if (!WheelSpeedControl_SetGainsQ10(
            &s_right_controller,
            right_kp_q10,
            right_ki_q10))
    {
        return false;
    }

    return true;
}

bool Chassis_SetWheelFeedforwardQ10(
    int32_t left_gain_q10,
    int16_t left_static_pwm,
    int32_t right_gain_q10,
    int16_t right_static_pwm)
{
    if (!s_initialized)
    {
        return false;
    }

    if (!WheelSpeedControl_SetFeedforwardQ10(
            &s_left_controller,
            left_gain_q10,
            left_static_pwm))
    {
        return false;
    }

    if (!WheelSpeedControl_SetFeedforwardQ10(
            &s_right_controller,
            right_gain_q10,
            right_static_pwm))
    {
        return false;
    }

    return true;
}

void Chassis_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    Chassis_ResetSpeedProfile();
    Chassis_ClearControlStatus();

    WheelSpeedControl_Reset(&s_left_controller);
    WheelSpeedControl_Reset(&s_right_controller);

    s_emergency_latched = true;
    s_status.emergency_latched = true;
    s_status.motion_mode = CHASSIS_MOTION_MODE_EMERGENCY;
    s_status.motion_stopped = true;

    BSP_Motor_BrakeAll();
}

bool Chassis_Update(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
    int32_t allowed_delta;
    bool left_delta_valid;
    bool right_delta_valid;
    BspEncoderSample_t encoder_sample;
    int16_t left_pwm;
    int16_t right_pwm;

    if ((!Chassis_IsEnabled()) || s_emergency_latched)
    {
        return false;
    }

    now_ms = HAL_GetTick();
    elapsed_ms = (uint32_t)(now_ms - s_last_control_ms);

    if (elapsed_ms < (uint32_t)CHASSIS_CONTROL_PERIOD_MS)
    {
        return false;
    }

    if (elapsed_ms > (uint32_t)CHASSIS_TIMING_OVERRUN_MS)
    {
        (void)BSP_Encoder_Sample(&encoder_sample);

        Chassis_Stop();
        s_status.encoder_sample_valid = false;
        s_status.dt_ms = (uint16_t)elapsed_ms;
        s_status.timestamp_ms = now_ms;
        s_status.timing_overrun_count++;
        s_status.control_sequence++;
        s_last_control_ms = now_ms;
        return false;
    }

    s_last_control_ms = now_ms;

    if (!BSP_Encoder_Sample(&encoder_sample))
    {
        Chassis_Stop();
        return false;
    }

    allowed_delta =
        Chassis_GetAllowedEncoderDelta((uint16_t)elapsed_ms);

    left_delta_valid =
        Chassis_AbsDelta(encoder_sample.left_delta) <=
        allowed_delta;

    right_delta_valid =
        Chassis_AbsDelta(encoder_sample.right_delta) <=
        allowed_delta;

    if ((!left_delta_valid) || (!right_delta_valid))
    {
        if (!left_delta_valid)
        {
            s_status.left_encoder_reject_count++;
        }

        if (!right_delta_valid)
        {
            s_status.right_encoder_reject_count++;
        }

        s_status.left_delta = encoder_sample.left_delta;
        s_status.right_delta = encoder_sample.right_delta;
        s_status.encoder_sample_valid = false;

        Chassis_Stop();
        s_status.dt_ms = (uint16_t)elapsed_ms;
        s_status.timestamp_ms = now_ms;
        s_status.control_sequence++;
        return false;
    }

    if (!Chassis_UpdateSpeedProfile((uint16_t)elapsed_ms))
    {
        Chassis_Stop();
        return false;
    }

    left_pwm = WheelSpeedControl_Update(
        &s_left_controller,
        encoder_sample.left_delta,
        (uint16_t)elapsed_ms);

    right_pwm = WheelSpeedControl_Update(
        &s_right_controller,
        encoder_sample.right_delta,
        (uint16_t)elapsed_ms);

    if (!BSP_Motor_Set(left_pwm, right_pwm))
    {
        Chassis_Stop();
        return false;
    }

    s_status.left_target_cps =
        WheelSpeedControl_GetTargetCps(&s_left_controller);
    s_status.right_target_cps =
        WheelSpeedControl_GetTargetCps(&s_right_controller);

    s_status.left_raw_measured_cps =
        WheelSpeedControl_GetRawMeasuredCps(&s_left_controller);
    s_status.right_raw_measured_cps =
        WheelSpeedControl_GetRawMeasuredCps(&s_right_controller);
    s_status.left_measured_cps =
        WheelSpeedControl_GetMeasuredCps(&s_left_controller);
    s_status.right_measured_cps =
        WheelSpeedControl_GetMeasuredCps(&s_right_controller);

    s_status.left_measured_mm_s =
        Chassis_CpsToMmps(s_status.left_measured_cps);
    s_status.right_measured_mm_s =
        Chassis_CpsToMmps(s_status.right_measured_cps);

    s_status.left_error_cps =
        WheelSpeedControl_GetErrorCps(&s_left_controller);
    s_status.right_error_cps =
        WheelSpeedControl_GetErrorCps(&s_right_controller);

    s_status.left_feedforward_pwm =
        WheelSpeedControl_GetFeedforwardPwm(&s_left_controller);
    s_status.right_feedforward_pwm =
        WheelSpeedControl_GetFeedforwardPwm(&s_right_controller);
    s_status.left_proportional_pwm =
        WheelSpeedControl_GetProportionalPwm(&s_left_controller);
    s_status.right_proportional_pwm =
        WheelSpeedControl_GetProportionalPwm(&s_right_controller);
    s_status.left_integral_pwm =
        WheelSpeedControl_GetIntegralPwm(&s_left_controller);
    s_status.right_integral_pwm =
        WheelSpeedControl_GetIntegralPwm(&s_right_controller);

    s_status.left_output_saturated =
        WheelSpeedControl_IsOutputSaturated(&s_left_controller);
    s_status.right_output_saturated =
        WheelSpeedControl_IsOutputSaturated(&s_right_controller);

    s_status.left_pwm = left_pwm;
    s_status.right_pwm = right_pwm;
    s_status.left_delta = encoder_sample.left_delta;
    s_status.right_delta = encoder_sample.right_delta;

    s_status.left_total = Chassis_AddSaturated(
        s_status.left_total,
        encoder_sample.left_delta);
    s_status.right_total = Chassis_AddSaturated(
        s_status.right_total,
        encoder_sample.right_delta);

    Chassis_UpdateStoppedState((uint16_t)elapsed_ms);

    s_status.encoder_sample_valid = true;
    s_status.dt_ms = (uint16_t)elapsed_ms;
    s_status.timestamp_ms = now_ms;
    s_status.control_sequence++;
    s_status.emergency_latched = s_emergency_latched;

    return true;
}

bool Chassis_GetStatus(ChassisStatus_t *status)
{
    if ((!s_initialized) || (status == NULL))
    {
        return false;
    }

    *status = s_status;
    return true;
}

const char *Chassis_MotionModeName(ChassisMotionMode_t mode)
{
    switch (mode)
    {
        case CHASSIS_MOTION_MODE_IDLE:
            return "IDLE";
        case CHASSIS_MOTION_MODE_DRIVE:
            return "DRIVE";
        case CHASSIS_MOTION_MODE_SOFT_STOP:
            return "SOFT";
        case CHASSIS_MOTION_MODE_FAST_STOP:
            return "FAST";
        case CHASSIS_MOTION_MODE_REVERSAL_STOP:
            return "REV";
        case CHASSIS_MOTION_MODE_EMERGENCY:
            return "EMG";
        default:
            return "UNKNOWN";
    }
}
