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

    /*
     * 向上取整：
     * ceil(max_cps × dt_ms / 1000) + margin。
     */
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
    /*
     * mm/s × 1000 = um/s
     * cps = um/s × counts/rev ÷ circumference_um
     */
    return Chassis_DivideRounded(
        (int64_t)speed_mm_s *
        (int64_t)BSP_ENCODER_COUNTS_PER_REV *
        1000LL,
        (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM);
}

int32_t Chassis_CpsToMmps(int32_t speed_cps)
{
    /*
     * mm/s = cps × circumference_um ÷ counts/rev ÷ 1000
     */
    return Chassis_DivideRounded(
        (int64_t)speed_cps *
        (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM,
        (int64_t)BSP_ENCODER_COUNTS_PER_REV *
        1000LL);
}

static void Chassis_ResetControllers(void)
{
    WheelSpeedControl_Reset(&s_left_controller);
    WheelSpeedControl_Reset(&s_right_controller);

    (void)WheelSpeedControl_SetTargetCps(
        &s_left_controller,
        s_status.left_target_cps);

    (void)WheelSpeedControl_SetTargetCps(
        &s_right_controller,
        s_status.right_target_cps);
}

bool Chassis_Init(void)
{
    WheelSpeedConfig_t left_config;
    WheelSpeedConfig_t right_config;

    s_initialized = false;
    s_enabled = false;
    memset(&s_status, 0, sizeof(s_status));

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

    if (!WheelSpeedControl_Init(&s_left_controller, &left_config))
    {
        return false;
    }

    if (!WheelSpeedControl_Init(&s_right_controller, &right_config))
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

    s_status.left_target_cps =
        Chassis_ClampCps((int64_t)left_cps);

    s_status.right_target_cps =
        Chassis_ClampCps((int64_t)right_cps);

    s_status.left_target_mm_s =
        Chassis_CpsToMmps(s_status.left_target_cps);

    s_status.right_target_mm_s =
        Chassis_CpsToMmps(s_status.right_target_cps);

    if (!WheelSpeedControl_SetTargetCps(
            &s_left_controller,
            s_status.left_target_cps))
    {
        return false;
    }

    if (!WheelSpeedControl_SetTargetCps(
            &s_right_controller,
            s_status.right_target_cps))
    {
        return false;
    }

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

    /*
     * 单侧差速量 = omega(rad/s) × track(mm) / 2
     * omega以mrad/s输入，因此除以2000。
     */
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

    s_status.left_target_mm_s = 0;
    s_status.right_target_mm_s = 0;
    s_status.left_target_cps = 0;
    s_status.right_target_cps = 0;

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
        /*
         * 严重超时后先同步并丢弃这段累计增量，
         * 避免把长时间累计值按单个控制周期处理。
         */
        (void)BSP_Encoder_Sample(&encoder_sample);

        Chassis_ResetControllers();
        BSP_Motor_BrakeAll();
        Chassis_ClearControlStatus();

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

        /*
         * BSP已经同步了硬件计数基准，因此本次异常不会在下一周期
         * 被重复读取。底盘层拒绝将异常增量加入有效里程。
         */
        s_status.left_delta = encoder_sample.left_delta;
        s_status.right_delta = encoder_sample.right_delta;
        s_status.encoder_sample_valid = false;

        s_status.left_raw_measured_cps = 0;
        s_status.right_raw_measured_cps = 0;
        s_status.left_measured_cps = 0;
        s_status.right_measured_cps = 0;
        s_status.left_measured_mm_s = 0;
        s_status.right_measured_mm_s = 0;
        s_status.left_error_cps = s_status.left_target_cps;
        s_status.right_error_cps = s_status.right_target_cps;

        Chassis_ResetControllers();
        BSP_Motor_BrakeAll();
        Chassis_ClearControlStatus();

        s_status.dt_ms = (uint16_t)elapsed_ms;
        s_status.timestamp_ms = now_ms;
        s_status.control_sequence++;

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

    /*
     * 只累计已经通过合理性检查的增量。
     * 避免外部接线故障导致底盘有效里程瞬间跳变。
     */
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
