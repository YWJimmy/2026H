#include "chassis.h"

#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "chassis_config.h"
#include "stm32f4xx_hal.h"
#include "wheel_speed_control.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static bool s_enabled = false;

static WheelSpeedController_t s_left_controller;
static WheelSpeedController_t s_right_controller;

static ChassisStatus_t s_status;
static uint32_t s_last_control_ms = 0U;

/* 最终命令与中间斜坡状态均使用count/s保存。 */
static int32_t s_left_command_cps = 0;
static int32_t s_right_command_cps = 0;
static int32_t s_forward_ramped_cps = 0;
static int32_t s_turn_ramped_cps = 0;

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


static int32_t Chassis_KeepCommandDirection(
    int32_t ramped_cps,
    int32_t command_cps)
{
    /*
     * 当最终命令明确为正/负时，中间目标不得短暂越过零点。
     * 这可避免转向斜坡快于前进斜坡时，非负巡线命令
     * 临时产生反向轮速。
     *
     * command=0时不强行钳位，允许当前速度按减速斜坡回零。
     */
    if ((command_cps > 0) && (ramped_cps < 0))
    {
        return 0;
    }

    if ((command_cps < 0) && (ramped_cps > 0))
    {
        return 0;
    }

    return ramped_cps;
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

static int32_t Chassis_ApproachRate(
    int32_t current,
    int32_t target,
    int32_t rate_cps_per_s,
    uint16_t dt_ms)
{
    int64_t scaled_step;
    int32_t maximum_step;
    int32_t difference;

    if (rate_cps_per_s <= 0)
    {
        return target;
    }

    /* 向上取整，避免5 ms周期下因整数截断停滞。 */
    scaled_step =
        (int64_t)rate_cps_per_s * (int64_t)dt_ms;

    maximum_step =
        (int32_t)((scaled_step + 999LL) / 1000LL);

    if (maximum_step < 1)
    {
        maximum_step = 1;
    }

    difference = target - current;

    if (difference > maximum_step)
    {
        return current + maximum_step;
    }

    if (difference < -maximum_step)
    {
        return current - maximum_step;
    }

    return target;
}

static int32_t Chassis_SelectForwardRateCps(
    int32_t current_cps,
    int32_t target_cps)
{
    bool same_direction;
    bool increasing_magnitude;

    if ((current_cps == 0) && (target_cps != 0))
    {
        return Chassis_MmpsToCps(
            CHASSIS_RAMP_FORWARD_ACCEL_MM_S2);
    }

    if (target_cps == 0)
    {
        return Chassis_MmpsToCps(
            CHASSIS_RAMP_FORWARD_DECEL_MM_S2);
    }

    same_direction =
        ((current_cps >= 0) && (target_cps >= 0)) ||
        ((current_cps <= 0) && (target_cps <= 0));

    if (!same_direction)
    {
        /* 换向先按减速斜率穿过0。 */
        return Chassis_MmpsToCps(
            CHASSIS_RAMP_FORWARD_DECEL_MM_S2);
    }

    increasing_magnitude =
        Chassis_AbsI32(target_cps) >
        Chassis_AbsI32(current_cps);

    return Chassis_MmpsToCps(
        increasing_magnitude
            ? CHASSIS_RAMP_FORWARD_ACCEL_MM_S2
            : CHASSIS_RAMP_FORWARD_DECEL_MM_S2);
}

static void Chassis_ResetSpeedRamp(void)
{
    s_left_command_cps = 0;
    s_right_command_cps = 0;
    s_forward_ramped_cps = 0;
    s_turn_ramped_cps = 0;

    s_status.left_command_mm_s = 0;
    s_status.right_command_mm_s = 0;
    s_status.left_command_cps = 0;
    s_status.right_command_cps = 0;

    s_status.left_target_mm_s = 0;
    s_status.right_target_mm_s = 0;
    s_status.left_target_cps = 0;
    s_status.right_target_cps = 0;
    s_status.speed_ramp_active = false;
}


static bool Chassis_UpdateSpeedRamp(uint16_t dt_ms)
{
    int32_t command_forward_cps;
    int32_t command_turn_cps;
    int32_t forward_rate_cps_per_s;
    int32_t turn_rate_cps_per_s;
    int32_t left_ramped_cps;
    int32_t right_ramped_cps;

    command_forward_cps =
        (s_left_command_cps + s_right_command_cps) / 2;

    command_turn_cps =
        (s_left_command_cps - s_right_command_cps) / 2;

    forward_rate_cps_per_s =
        Chassis_SelectForwardRateCps(
            s_forward_ramped_cps,
            command_forward_cps);

    turn_rate_cps_per_s =
        Chassis_MmpsToCps(
            CHASSIS_RAMP_TURN_SLEW_MM_S2);

    s_forward_ramped_cps = Chassis_ApproachRate(
        s_forward_ramped_cps,
        command_forward_cps,
        forward_rate_cps_per_s,
        dt_ms);

    s_turn_ramped_cps = Chassis_ApproachRate(
        s_turn_ramped_cps,
        command_turn_cps,
        turn_rate_cps_per_s,
        dt_ms);

    left_ramped_cps = Chassis_ClampCps(
        (int64_t)s_forward_ramped_cps +
        (int64_t)s_turn_ramped_cps);

    right_ramped_cps = Chassis_ClampCps(
        (int64_t)s_forward_ramped_cps -
        (int64_t)s_turn_ramped_cps);

    left_ramped_cps = Chassis_KeepCommandDirection(
        left_ramped_cps,
        s_left_command_cps);

    right_ramped_cps = Chassis_KeepCommandDirection(
        right_ramped_cps,
        s_right_command_cps);

    /*
     * 两轮限幅后重新同步内部前进/转向状态，
     * 防止内部状态长期超出可实现范围。
     */
    s_forward_ramped_cps =
        (left_ramped_cps + right_ramped_cps) / 2;

    s_turn_ramped_cps =
        (left_ramped_cps - right_ramped_cps) / 2;

    s_status.left_target_cps = left_ramped_cps;
    s_status.right_target_cps = right_ramped_cps;
    s_status.left_target_mm_s =
        Chassis_CpsToMmps(left_ramped_cps);
    s_status.right_target_mm_s =
        Chassis_CpsToMmps(right_ramped_cps);

    s_status.speed_ramp_active =
        (left_ramped_cps != s_left_command_cps) ||
        (right_ramped_cps != s_right_command_cps);

    if (!WheelSpeedControl_SetTargetCps(
            &s_left_controller,
            left_ramped_cps))
    {
        return false;
    }

    if (!WheelSpeedControl_SetTargetCps(
            &s_right_controller,
            right_ramped_cps))
    {
        return false;
    }

    return true;
}

bool Chassis_Init(void)
{
    WheelSpeedConfig_t left_config;
    WheelSpeedConfig_t right_config;

    s_initialized = false;
    s_enabled = false;
    memset(&s_status, 0, sizeof(s_status));
    Chassis_ResetSpeedRamp();

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

    s_last_control_ms = HAL_GetTick();
    s_initialized = true;
    s_status.initialized = true;

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

        Chassis_ResetSpeedRamp();
        WheelSpeedControl_Reset(&s_left_controller);
        WheelSpeedControl_Reset(&s_right_controller);

        if (!BSP_Motor_Enable(true))
        {
            return false;
        }

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

bool Chassis_SetWheelSpeedCps(int32_t left_cps,
                              int32_t right_cps)
{
    if (!s_initialized)
    {
        return false;
    }

    s_left_command_cps =
        Chassis_ClampCps((int64_t)left_cps);

    s_right_command_cps =
        Chassis_ClampCps((int64_t)right_cps);

    s_status.left_command_cps = s_left_command_cps;
    s_status.right_command_cps = s_right_command_cps;
    s_status.left_command_mm_s =
        Chassis_CpsToMmps(s_left_command_cps);
    s_status.right_command_mm_s =
        Chassis_CpsToMmps(s_right_command_cps);

    s_status.speed_ramp_active =
        (s_status.left_target_cps != s_left_command_cps) ||
        (s_status.right_target_cps != s_right_command_cps);

    return true;
}

bool Chassis_SetWheelSpeedMmps(int32_t left_mm_s,
                               int32_t right_mm_s)
{
    int32_t clamped_left_mm_s;
    int32_t clamped_right_mm_s;

    if (!s_initialized)
    {
        return false;
    }

    clamped_left_mm_s =
        Chassis_ClampMmps((int64_t)left_mm_s);

    clamped_right_mm_s =
        Chassis_ClampMmps((int64_t)right_mm_s);

    return Chassis_SetWheelSpeedCps(
        Chassis_MmpsToCps(clamped_left_mm_s),
        Chassis_MmpsToCps(clamped_right_mm_s));
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

    left_mm_s =
        (int64_t)linear_mm_s - turning_mm_s;

    right_mm_s =
        (int64_t)linear_mm_s + turning_mm_s;

    return Chassis_SetWheelSpeedMmps(
        Chassis_ClampMmps(left_mm_s),
        Chassis_ClampMmps(right_mm_s));
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

    Chassis_ResetSpeedRamp();
    Chassis_ClearControlStatus();

    WheelSpeedControl_Reset(&s_left_controller);
    WheelSpeedControl_Reset(&s_right_controller);

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

    if (!Chassis_IsEnabled())
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

    if (!Chassis_UpdateSpeedRamp((uint16_t)elapsed_ms))
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

    s_status.left_target_mm_s =
        Chassis_CpsToMmps(s_status.left_target_cps);

    s_status.right_target_mm_s =
        Chassis_CpsToMmps(s_status.right_target_cps);

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

    s_status.encoder_sample_valid = true;
    s_status.dt_ms = (uint16_t)elapsed_ms;
    s_status.timestamp_ms = now_ms;
    s_status.control_sequence++;

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
