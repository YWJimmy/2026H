#include "chassis_speed_profile.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define PROFILE_Q_SHIFT 16
#define PROFILE_Q_ONE   ((int32_t)65536)

static int32_t Profile_ClampI32(int64_t value,
                                int32_t minimum,
                                int32_t maximum)
{
    if (value < (int64_t)minimum)
    {
        return minimum;
    }

    if (value > (int64_t)maximum)
    {
        return maximum;
    }

    return (int32_t)value;
}

static int32_t Profile_AbsI32(int32_t value)
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

static int32_t Profile_SignI32(int32_t value)
{
    if (value > 0)
    {
        return 1;
    }

    if (value < 0)
    {
        return -1;
    }

    return 0;
}

static int32_t Profile_ToQ16(int32_t value)
{
    return Profile_ClampI32(
        (int64_t)value * (int64_t)PROFILE_Q_ONE,
        INT32_MIN,
        INT32_MAX);
}

static int32_t Profile_FromQ16(int32_t value_q16)
{
    int64_t value = (int64_t)value_q16;

    if (value >= 0)
    {
        return (int32_t)(
            (value + (PROFILE_Q_ONE / 2)) >> PROFILE_Q_SHIFT);
    }

    return (int32_t)(-(((-value) + (PROFILE_Q_ONE / 2)) >>
                       PROFILE_Q_SHIFT));
}

static int32_t Profile_ApproachQ16(int32_t current_q16,
                                   int32_t target_q16,
                                   int32_t step_q16)
{
    int64_t difference;

    if (step_q16 <= 0)
    {
        return target_q16;
    }

    difference = (int64_t)target_q16 - (int64_t)current_q16;

    if (difference > (int64_t)step_q16)
    {
        return current_q16 + step_q16;
    }

    if (difference < -(int64_t)step_q16)
    {
        return current_q16 - step_q16;
    }

    return target_q16;
}

static int32_t Profile_StepFromRateQ16(int32_t rate_q16_per_s,
                                       uint16_t dt_ms)
{
    int64_t step;

    if ((rate_q16_per_s <= 0) || (dt_ms == 0U))
    {
        return 0;
    }

    step = ((int64_t)rate_q16_per_s * (int64_t)dt_ms + 999LL) /
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

static int32_t Profile_BrakingDeltaSpeedQ16(int32_t acceleration_q16,
                                             int32_t jerk_q16)
{
    int64_t acceleration_abs;
    int64_t numerator;
    int64_t denominator;
    int64_t result;

    if (jerk_q16 <= 0)
    {
        return 0;
    }

    acceleration_abs = (int64_t)Profile_AbsI32(acceleration_q16);
    numerator = acceleration_abs * acceleration_abs;
    denominator = 2LL * (int64_t)jerk_q16;
    result = numerator / denominator;

    if (result > INT32_MAX)
    {
        return INT32_MAX;
    }

    return (int32_t)result;
}

static bool Profile_IsMagnitudeIncreasing(int32_t current_q16,
                                          int32_t target_q16)
{
    int32_t current_sign = Profile_SignI32(current_q16);
    int32_t target_sign = Profile_SignI32(target_q16);

    if (target_sign == 0)
    {
        return false;
    }

    if ((current_sign == 0) || (current_sign == target_sign))
    {
        return Profile_AbsI32(target_q16) >
               Profile_AbsI32(current_q16);
    }

    return false;
}

/*
 * 单轴在线限跃度更新。
 *
 * 速度、加速度均使用Q16；先限制加速度变化，再用梯形积分更新速度。
 * 接近目标时根据a^2/(2J)提前卸载加速度，防止明显越过目标。
 */
static bool Profile_UpdateAxis(
    ChassisSpeedProfileAxis_t *axis,
    int32_t target_speed_q16,
    int32_t accel_limit_mm_s2,
    int32_t decel_limit_mm_s2,
    int32_t accel_jerk_mm_s3,
    int32_t decel_jerk_mm_s3,
    int32_t speed_snap_mm_s,
    int32_t accel_snap_mm_s2,
    uint16_t dt_ms)
{
    int32_t old_speed_q16;
    int32_t old_accel_q16;
    int32_t speed_error_q16;
    int32_t speed_error_abs_q16;
    int32_t error_sign;
    int32_t acceleration_limit_q16;
    int32_t jerk_q16;
    int32_t desired_acceleration_q16;
    int32_t acceleration_step_q16;
    int32_t new_acceleration_q16;
    int32_t braking_delta_q16;
    int32_t speed_snap_q16;
    int32_t accel_snap_q16;
    int64_t average_acceleration_q16;
    int64_t speed_step_q16;
    int32_t new_speed_q16;
    int32_t new_error_q16;
    bool increasing_magnitude;
    bool acceleration_toward_target;
    bool crossed_target;

    if ((axis == NULL) || (dt_ms == 0U))
    {
        return false;
    }

    old_speed_q16 = axis->speed_q16;
    old_accel_q16 = axis->acceleration_q16;
    speed_error_q16 = target_speed_q16 - old_speed_q16;
    speed_error_abs_q16 = Profile_AbsI32(speed_error_q16);
    speed_snap_q16 = Profile_ToQ16(speed_snap_mm_s);
    accel_snap_q16 = Profile_ToQ16(accel_snap_mm_s2);

    if ((speed_error_abs_q16 <= speed_snap_q16) &&
        (Profile_AbsI32(old_accel_q16) <= accel_snap_q16))
    {
        axis->speed_q16 = target_speed_q16;
        axis->acceleration_q16 = 0;
        return false;
    }

    increasing_magnitude = Profile_IsMagnitudeIncreasing(
        old_speed_q16,
        target_speed_q16);

    acceleration_limit_q16 = Profile_ToQ16(
        increasing_magnitude ? accel_limit_mm_s2 : decel_limit_mm_s2);

    jerk_q16 = Profile_ToQ16(
        increasing_magnitude ? accel_jerk_mm_s3 : decel_jerk_mm_s3);

    error_sign = Profile_SignI32(speed_error_q16);
    desired_acceleration_q16 = error_sign * acceleration_limit_q16;

    acceleration_toward_target =
        (error_sign != 0) &&
        (Profile_SignI32(old_accel_q16) == error_sign);

    braking_delta_q16 = acceleration_toward_target ?
        Profile_BrakingDeltaSpeedQ16(old_accel_q16, jerk_q16) :
        0;

    if (acceleration_toward_target &&
        (speed_error_abs_q16 <=
         (braking_delta_q16 + speed_snap_q16)))
    {
        desired_acceleration_q16 = 0;
    }

    acceleration_step_q16 = Profile_StepFromRateQ16(
        jerk_q16,
        dt_ms);

    new_acceleration_q16 = Profile_ApproachQ16(
        old_accel_q16,
        desired_acceleration_q16,
        acceleration_step_q16);

    average_acceleration_q16 =
        ((int64_t)old_accel_q16 +
         (int64_t)new_acceleration_q16) /
        2LL;

    speed_step_q16 =
        (average_acceleration_q16 * (int64_t)dt_ms) /
        1000LL;

    new_speed_q16 = Profile_ClampI32(
        (int64_t)old_speed_q16 + speed_step_q16,
        INT32_MIN,
        INT32_MAX);

    new_error_q16 = target_speed_q16 - new_speed_q16;
    crossed_target =
        (speed_error_q16 != 0) &&
        (Profile_SignI32(new_error_q16) !=
         Profile_SignI32(speed_error_q16));

    if (crossed_target ||
        ((Profile_AbsI32(new_error_q16) <= speed_snap_q16) &&
         (Profile_AbsI32(new_acceleration_q16) <= accel_snap_q16)))
    {
        axis->speed_q16 = target_speed_q16;
        axis->acceleration_q16 = 0;
        return false;
    }

    axis->speed_q16 = new_speed_q16;
    axis->acceleration_q16 = new_acceleration_q16;
    return true;
}

static void Profile_SyncWheelAxesFromOutput(
    ChassisSpeedProfile_t *profile,
    bool reset_acceleration)
{
    profile->left_axis.speed_q16 = profile->output_left_q16;
    profile->right_axis.speed_q16 = profile->output_right_q16;

    if (reset_acceleration)
    {
        profile->left_axis.acceleration_q16 = 0;
        profile->right_axis.acceleration_q16 = 0;
    }
}

static bool Profile_HasDirectionConflict(
    const ChassisSpeedProfile_t *profile,
    int32_t left_command_q16,
    int32_t right_command_q16)
{
    int32_t speed_snap_q16;
    bool left_conflict;
    bool right_conflict;

    speed_snap_q16 = Profile_ToQ16(
        profile->config.speed_snap_mm_s);

    left_conflict =
        ((profile->output_left_q16 > speed_snap_q16) &&
         (left_command_q16 < 0)) ||
        ((profile->output_left_q16 < -speed_snap_q16) &&
         (left_command_q16 > 0));

    right_conflict =
        ((profile->output_right_q16 > speed_snap_q16) &&
         (right_command_q16 < 0)) ||
        ((profile->output_right_q16 < -speed_snap_q16) &&
         (right_command_q16 > 0));

    return left_conflict || right_conflict;
}

static void Profile_CaptureStopStart(
    ChassisSpeedProfile_t *profile)
{
    int32_t left_abs_q16;
    int32_t right_abs_q16;

    profile->stop_start_left_q16 = profile->output_left_q16;
    profile->stop_start_right_q16 = profile->output_right_q16;

    left_abs_q16 = Profile_AbsI32(
        profile->stop_start_left_q16);
    right_abs_q16 = Profile_AbsI32(
        profile->stop_start_right_q16);

    profile->stop_start_reference_q16 =
        (left_abs_q16 > right_abs_q16) ?
        left_abs_q16 : right_abs_q16;

    profile->stop_axis.speed_q16 =
        profile->stop_start_reference_q16;
    profile->stop_axis.acceleration_q16 = 0;
    profile->active =
        profile->stop_start_reference_q16 > 0;
}

/*
 * 判断本周期是“整体前进速度变化”还是“差速转向变化”。
 * 转向误差占主导时使用更快的转向约束；其余情况使用较慢的前进约束。
 */
static bool Profile_IsTurnDominant(
    const ChassisSpeedProfile_t *profile)
{
    int32_t left_error_q16;
    int32_t right_error_q16;
    int32_t forward_error_q16;
    int32_t turn_error_q16;
    int32_t snap_q16;

    left_error_q16 =
        profile->command_left_q16 - profile->left_axis.speed_q16;
    right_error_q16 =
        profile->command_right_q16 - profile->right_axis.speed_q16;

    forward_error_q16 =
        (left_error_q16 + right_error_q16) / 2;
    turn_error_q16 =
        (left_error_q16 - right_error_q16) / 2;
    snap_q16 = Profile_ToQ16(profile->config.speed_snap_mm_s);

    return Profile_AbsI32(turn_error_q16) >
           (Profile_AbsI32(forward_error_q16) + snap_q16);
}

static bool Profile_UpdateDrive(ChassisSpeedProfile_t *profile,
                                uint16_t dt_ms)
{
    int32_t accel_limit_mm_s2;
    int32_t decel_limit_mm_s2;
    int32_t accel_jerk_mm_s3;
    int32_t decel_jerk_mm_s3;
    int32_t max_q16;
    bool left_active;
    bool right_active;
    bool turn_dominant;

    turn_dominant = Profile_IsTurnDominant(profile);

    if (turn_dominant)
    {
        accel_limit_mm_s2 = profile->config.turn_accel_mm_s2;
        decel_limit_mm_s2 = profile->config.turn_accel_mm_s2;
        accel_jerk_mm_s3 = profile->config.turn_jerk_mm_s3;
        decel_jerk_mm_s3 = profile->config.turn_jerk_mm_s3;
    }
    else
    {
        accel_limit_mm_s2 = profile->config.forward_accel_mm_s2;
        decel_limit_mm_s2 = profile->config.forward_decel_mm_s2;
        accel_jerk_mm_s3 =
            profile->config.forward_accel_jerk_mm_s3;
        decel_jerk_mm_s3 =
            profile->config.forward_decel_jerk_mm_s3;
    }

    left_active = Profile_UpdateAxis(
        &profile->left_axis,
        profile->command_left_q16,
        accel_limit_mm_s2,
        decel_limit_mm_s2,
        accel_jerk_mm_s3,
        decel_jerk_mm_s3,
        profile->config.speed_snap_mm_s,
        profile->config.accel_snap_mm_s2,
        dt_ms);

    right_active = Profile_UpdateAxis(
        &profile->right_axis,
        profile->command_right_q16,
        accel_limit_mm_s2,
        decel_limit_mm_s2,
        accel_jerk_mm_s3,
        decel_jerk_mm_s3,
        profile->config.speed_snap_mm_s,
        profile->config.accel_snap_mm_s2,
        dt_ms);

    max_q16 = Profile_ToQ16(profile->config.max_wheel_speed_mm_s);
    profile->left_axis.speed_q16 = Profile_ClampI32(
        profile->left_axis.speed_q16,
        -max_q16,
        max_q16);
    profile->right_axis.speed_q16 = Profile_ClampI32(
        profile->right_axis.speed_q16,
        -max_q16,
        max_q16);

    profile->output_left_q16 = profile->left_axis.speed_q16;
    profile->output_right_q16 = profile->right_axis.speed_q16;
    profile->active = left_active || right_active;

    if (!profile->active)
    {
        profile->output_left_q16 = profile->command_left_q16;
        profile->output_right_q16 = profile->command_right_q16;
        Profile_SyncWheelAxesFromOutput(profile, true);
    }

    return profile->active;
}

static bool Profile_UpdateStop(ChassisSpeedProfile_t *profile,
                               uint16_t dt_ms)
{
    int32_t decel_mm_s2;
    int32_t jerk_mm_s3;
    bool stop_active;

    if (profile->custom_stop_active)
    {
        decel_mm_s2 = profile->custom_stop_decel_mm_s2;
        jerk_mm_s3 = profile->custom_stop_jerk_mm_s3;
    }
    else if (profile->mode == CHASSIS_SPEED_PROFILE_MODE_FAST_STOP)
    {
        decel_mm_s2 = profile->config.fast_stop_decel_mm_s2;
        jerk_mm_s3 = profile->config.fast_stop_jerk_mm_s3;
    }
    else if (profile->mode ==
             CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP)
    {
        decel_mm_s2 = profile->config.forward_decel_mm_s2;
        jerk_mm_s3 = profile->config.forward_decel_jerk_mm_s3;
    }
    else
    {
        decel_mm_s2 = profile->config.soft_stop_decel_mm_s2;
        jerk_mm_s3 = profile->config.soft_stop_jerk_mm_s3;
    }

    stop_active = Profile_UpdateAxis(
        &profile->stop_axis,
        0,
        decel_mm_s2,
        decel_mm_s2,
        jerk_mm_s3,
        jerk_mm_s3,
        profile->config.speed_snap_mm_s,
        profile->config.accel_snap_mm_s2,
        dt_ms);

    if ((profile->stop_start_reference_q16 <= 0) || !stop_active)
    {
        profile->output_left_q16 = 0;
        profile->output_right_q16 = 0;
        profile->stop_axis.speed_q16 = 0;
        profile->stop_axis.acceleration_q16 = 0;
        Profile_SyncWheelAxesFromOutput(profile, true);

        if (profile->mode ==
            CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP)
        {
            profile->command_left_q16 = profile->pending_left_q16;
            profile->command_right_q16 = profile->pending_right_q16;
            profile->mode = CHASSIS_SPEED_PROFILE_MODE_DRIVE;
            profile->active = true;
            return true;
        }

        profile->active = false;
        return false;
    }

    profile->output_left_q16 = (int32_t)(
        ((int64_t)profile->stop_start_left_q16 *
         (int64_t)profile->stop_axis.speed_q16) /
        (int64_t)profile->stop_start_reference_q16);

    profile->output_right_q16 = (int32_t)(
        ((int64_t)profile->stop_start_right_q16 *
         (int64_t)profile->stop_axis.speed_q16) /
        (int64_t)profile->stop_start_reference_q16);

    Profile_SyncWheelAxesFromOutput(profile, true);
    profile->active = true;
    return true;
}

bool ChassisSpeedProfile_Init(
    ChassisSpeedProfile_t *profile,
    const ChassisSpeedProfileConfig_t *config)
{
    if ((profile == NULL) ||
        (config == NULL) ||
        (config->max_wheel_speed_mm_s <= 0) ||
        (config->forward_accel_mm_s2 <= 0) ||
        (config->forward_decel_mm_s2 <= 0) ||
        (config->forward_accel_jerk_mm_s3 <= 0) ||
        (config->forward_decel_jerk_mm_s3 <= 0) ||
        (config->turn_accel_mm_s2 <= 0) ||
        (config->turn_jerk_mm_s3 <= 0) ||
        (config->soft_stop_decel_mm_s2 <= 0) ||
        (config->soft_stop_jerk_mm_s3 <= 0) ||
        (config->fast_stop_decel_mm_s2 <= 0) ||
        (config->fast_stop_jerk_mm_s3 <= 0) ||
        (config->speed_snap_mm_s < 0) ||
        (config->accel_snap_mm_s2 < 0))
    {
        return false;
    }

    memset(profile, 0, sizeof(*profile));
    profile->config = *config;
    profile->initialized = true;
    profile->mode = CHASSIS_SPEED_PROFILE_MODE_IDLE;
    return true;
}

void ChassisSpeedProfile_Reset(ChassisSpeedProfile_t *profile)
{
    ChassisSpeedProfileConfig_t config;
    bool initialized;

    if (profile == NULL)
    {
        return;
    }

    config = profile->config;
    initialized = profile->initialized;
    memset(profile, 0, sizeof(*profile));
    profile->config = config;
    profile->initialized = initialized;
    profile->mode = CHASSIS_SPEED_PROFILE_MODE_IDLE;
}

bool ChassisSpeedProfile_SetCommandMmps(
    ChassisSpeedProfile_t *profile,
    int32_t left_mm_s,
    int32_t right_mm_s)
{
    int32_t maximum;
    int32_t left_command_q16;
    int32_t right_command_q16;

    if ((profile == NULL) || (!profile->initialized))
    {
        return false;
    }

    maximum = profile->config.max_wheel_speed_mm_s;
    left_mm_s = Profile_ClampI32(left_mm_s, -maximum, maximum);
    right_mm_s = Profile_ClampI32(right_mm_s, -maximum, maximum);

    if ((left_mm_s == 0) && (right_mm_s == 0))
    {
        if ((profile->mode == CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP) ||
            (profile->mode == CHASSIS_SPEED_PROFILE_MODE_FAST_STOP))
        {
            return true;
        }

        return ChassisSpeedProfile_RequestStop(
            profile,
            CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP);
    }

    left_command_q16 = Profile_ToQ16(left_mm_s);
    right_command_q16 = Profile_ToQ16(right_mm_s);

    profile->pending_left_q16 = left_command_q16;
    profile->pending_right_q16 = right_command_q16;

    if (Profile_HasDirectionConflict(
            profile,
            left_command_q16,
            right_command_q16))
    {
        profile->command_left_q16 = left_command_q16;
        profile->command_right_q16 = right_command_q16;

        if (profile->mode !=
            CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP)
        {
            Profile_CaptureStopStart(profile);
            profile->mode =
                CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP;
        }

        return true;
    }

    if ((profile->mode == CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP) ||
        (profile->mode == CHASSIS_SPEED_PROFILE_MODE_FAST_STOP) ||
        (profile->mode == CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP) ||
        (profile->mode == CHASSIS_SPEED_PROFILE_MODE_IDLE))
    {
        Profile_SyncWheelAxesFromOutput(profile, true);
    }

    profile->command_left_q16 = left_command_q16;
    profile->command_right_q16 = right_command_q16;
    profile->mode = CHASSIS_SPEED_PROFILE_MODE_DRIVE;
    profile->active =
        (profile->output_left_q16 != profile->command_left_q16) ||
        (profile->output_right_q16 != profile->command_right_q16);
    return true;
}

bool ChassisSpeedProfile_RequestStop(
    ChassisSpeedProfile_t *profile,
    ChassisSpeedProfileMode_t stop_mode)
{
    if ((profile == NULL) ||
        (!profile->initialized) ||
        ((stop_mode != CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP) &&
         (stop_mode != CHASSIS_SPEED_PROFILE_MODE_FAST_STOP)))
    {
        return false;
    }

    if ((profile->mode == CHASSIS_SPEED_PROFILE_MODE_FAST_STOP) &&
        (stop_mode == CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP))
    {
        return true;
    }

    if (profile->mode == stop_mode)
    {
        return true;
    }

    profile->command_left_q16 = 0;
    profile->command_right_q16 = 0;
    profile->pending_left_q16 = 0;
    profile->pending_right_q16 = 0;
    profile->custom_stop_active = false;

    Profile_CaptureStopStart(profile);
    profile->mode = stop_mode;

    if (!profile->active)
    {
        profile->output_left_q16 = 0;
        profile->output_right_q16 = 0;
    }

    return true;
}

bool ChassisSpeedProfile_RequestStopWithDecel(
    ChassisSpeedProfile_t *profile,
    int32_t decel_mm_s2,
    int32_t jerk_mm_s3)
{
    if ((profile == NULL) || (!profile->initialized) ||
        (decel_mm_s2 <= 0) || (jerk_mm_s3 <= 0))
    {
        return false;
    }

    profile->command_left_q16 = 0;
    profile->command_right_q16 = 0;
    profile->pending_left_q16 = 0;
    profile->pending_right_q16 = 0;
    profile->custom_stop_decel_mm_s2 = decel_mm_s2;
    profile->custom_stop_jerk_mm_s3 = jerk_mm_s3;
    profile->custom_stop_active = true;

    Profile_CaptureStopStart(profile);
    profile->mode = CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP;

    if (!profile->active)
    {
        profile->output_left_q16 = 0;
        profile->output_right_q16 = 0;
    }

    return true;
}

bool ChassisSpeedProfile_SynchronizeOutputMmps(
    ChassisSpeedProfile_t *profile,
    int32_t left_mm_s,
    int32_t right_mm_s)
{
    int32_t maximum;

    if ((profile == NULL) || (!profile->initialized))
    {
        return false;
    }

    maximum = profile->config.max_wheel_speed_mm_s;
    left_mm_s = Profile_ClampI32(left_mm_s, -maximum, maximum);
    right_mm_s = Profile_ClampI32(right_mm_s, -maximum, maximum);

    profile->output_left_q16 = Profile_ToQ16(left_mm_s);
    profile->output_right_q16 = Profile_ToQ16(right_mm_s);
    profile->command_left_q16 = profile->output_left_q16;
    profile->command_right_q16 = profile->output_right_q16;
    profile->pending_left_q16 = profile->output_left_q16;
    profile->pending_right_q16 = profile->output_right_q16;

    profile->left_axis.speed_q16 = profile->output_left_q16;
    profile->right_axis.speed_q16 = profile->output_right_q16;
    profile->left_axis.acceleration_q16 = 0;
    profile->right_axis.acceleration_q16 = 0;

    profile->stop_axis.speed_q16 = 0;
    profile->stop_axis.acceleration_q16 = 0;
    profile->stop_start_left_q16 = 0;
    profile->stop_start_right_q16 = 0;
    profile->stop_start_reference_q16 = 0;

    profile->mode = ((left_mm_s == 0) && (right_mm_s == 0)) ?
        CHASSIS_SPEED_PROFILE_MODE_IDLE :
        CHASSIS_SPEED_PROFILE_MODE_DRIVE;
    profile->active = false;
    return true;
}

bool ChassisSpeedProfile_Update(
    ChassisSpeedProfile_t *profile,
    uint16_t dt_ms)
{
    if ((profile == NULL) ||
        (!profile->initialized) ||
        (dt_ms == 0U))
    {
        return false;
    }

    switch (profile->mode)
    {
        case CHASSIS_SPEED_PROFILE_MODE_DRIVE:
            (void)Profile_UpdateDrive(profile, dt_ms);
            break;

        case CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP:
        case CHASSIS_SPEED_PROFILE_MODE_FAST_STOP:
        case CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP:
            (void)Profile_UpdateStop(profile, dt_ms);
            break;

        case CHASSIS_SPEED_PROFILE_MODE_IDLE:
        default:
            profile->output_left_q16 = 0;
            profile->output_right_q16 = 0;
            profile->active = false;
            break;
    }

    return true;
}

bool ChassisSpeedProfile_GetStatus(
    const ChassisSpeedProfile_t *profile,
    ChassisSpeedProfileStatus_t *status)
{
    int32_t forward_speed_q16;
    int32_t turn_speed_q16;
    int32_t forward_accel_q16;
    int32_t turn_accel_q16;

    if ((profile == NULL) ||
        (!profile->initialized) ||
        (status == NULL))
    {
        return false;
    }

    forward_speed_q16 =
        (profile->output_left_q16 + profile->output_right_q16) / 2;
    turn_speed_q16 =
        (profile->output_left_q16 - profile->output_right_q16) / 2;
    forward_accel_q16 =
        (profile->left_axis.acceleration_q16 +
         profile->right_axis.acceleration_q16) /
        2;
    turn_accel_q16 =
        (profile->left_axis.acceleration_q16 -
         profile->right_axis.acceleration_q16) /
        2;

    status->mode = profile->mode;
    status->active = profile->active;
    status->stopped =
        (!profile->active) &&
        (profile->output_left_q16 == 0) &&
        (profile->output_right_q16 == 0);

    status->command_left_mm_s =
        Profile_FromQ16(profile->command_left_q16);
    status->command_right_mm_s =
        Profile_FromQ16(profile->command_right_q16);
    status->output_left_mm_s =
        Profile_FromQ16(profile->output_left_q16);
    status->output_right_mm_s =
        Profile_FromQ16(profile->output_right_q16);

    status->forward_speed_mm_s =
        Profile_FromQ16(forward_speed_q16);
    status->turn_speed_mm_s =
        Profile_FromQ16(turn_speed_q16);
    status->forward_accel_mm_s2 =
        Profile_FromQ16(forward_accel_q16);
    status->turn_accel_mm_s2 =
        Profile_FromQ16(turn_accel_q16);
    status->stop_reference_mm_s =
        Profile_FromQ16(profile->stop_axis.speed_q16);
    status->stop_accel_mm_s2 =
        Profile_FromQ16(profile->stop_axis.acceleration_q16);
    return true;
}

const char *ChassisSpeedProfile_ModeName(
    ChassisSpeedProfileMode_t mode)
{
    switch (mode)
    {
        case CHASSIS_SPEED_PROFILE_MODE_IDLE:
            return "IDLE";
        case CHASSIS_SPEED_PROFILE_MODE_DRIVE:
            return "DRIVE";
        case CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP:
            return "SOFT";
        case CHASSIS_SPEED_PROFILE_MODE_FAST_STOP:
            return "FAST";
        case CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP:
            return "REV";
        default:
            return "UNKNOWN";
    }
}
