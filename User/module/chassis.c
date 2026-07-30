#include "chassis.h"

#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "chassis_config.h"
#include "stm32f4xx_hal.h"
#include "wheel_speed_control.h"

#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static bool s_enabled = false;

static WheelSpeedController_t s_left_controller;
static WheelSpeedController_t s_right_controller;

static ChassisStatus_t s_status;
static uint32_t s_last_control_ms = 0U;

static int32_t Chassis_ClampSpeedMmps(int64_t speed_mm_s)
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

static int32_t Chassis_MmpsToCps(int32_t speed_mm_s)
{
    /*
     * mm/s × 1000 = um/s
     * cps = um/s × counts/rev ÷ circumference_um
     */
    return (int32_t)(
        ((int64_t)speed_mm_s *
         (int64_t)BSP_ENCODER_COUNTS_PER_REV *
         1000LL) /
        (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM);
}

static int32_t Chassis_DeltaToMmps(int16_t delta, uint16_t dt_ms)
{
    if (dt_ms == 0U)
    {
        return 0;
    }

    /*
     * delta × circumference_um / counts_per_rev / dt_ms
     * 单位为 um/ms，数值等同于 mm/s。
     */
    return (int32_t)(
        ((int64_t)delta *
         (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM) /
        ((int64_t)BSP_ENCODER_COUNTS_PER_REV *
         (int64_t)dt_ms));
}

static void Chassis_ResetControllers(void)
{
    WheelSpeedControl_Reset(&s_left_controller);
    WheelSpeedControl_Reset(&s_right_controller);

    (void)WheelSpeedControl_SetTargetCps(
        &s_left_controller,
        Chassis_MmpsToCps(s_status.left_target_mm_s));

    (void)WheelSpeedControl_SetTargetCps(
        &s_right_controller,
        Chassis_MmpsToCps(s_status.right_target_mm_s));
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
    left_config.pwm_limit = CHASSIS_PWM_LIMIT;
    left_config.min_drive_pwm = WHEEL_SPEED_LEFT_MIN_DRIVE_PWM;
    left_config.nominal_period_ms = CHASSIS_CONTROL_PERIOD_MS;

    right_config.kp_q10 = WHEEL_SPEED_RIGHT_KP_Q10;
    right_config.ki_q10 = WHEEL_SPEED_RIGHT_KI_Q10;
    right_config.pwm_limit = CHASSIS_PWM_LIMIT;
    right_config.min_drive_pwm = WHEEL_SPEED_RIGHT_MIN_DRIVE_PWM;
    right_config.nominal_period_ms = CHASSIS_CONTROL_PERIOD_MS;

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

        s_status.left_target_mm_s = 0;
        s_status.right_target_mm_s = 0;
        s_status.left_target_cps = 0;
        s_status.right_target_cps = 0;
        s_status.left_measured_mm_s = 0;
        s_status.right_measured_mm_s = 0;
        s_status.left_measured_cps = 0;
        s_status.right_measured_cps = 0;
        s_status.left_pwm = 0;
        s_status.right_pwm = 0;
        s_status.left_delta = 0;
        s_status.right_delta = 0;
        s_status.left_total = 0;
        s_status.right_total = 0;
        s_status.dt_ms = 0U;

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

bool Chassis_SetWheelSpeedMmps(int32_t left_mm_s,
                               int32_t right_mm_s)
{
    if (!s_initialized)
    {
        return false;
    }

    s_status.left_target_mm_s =
        Chassis_ClampSpeedMmps((int64_t)left_mm_s);

    s_status.right_target_mm_s =
        Chassis_ClampSpeedMmps((int64_t)right_mm_s);

    s_status.left_target_cps =
        Chassis_MmpsToCps(s_status.left_target_mm_s);

    s_status.right_target_cps =
        Chassis_MmpsToCps(s_status.right_target_mm_s);

    (void)WheelSpeedControl_SetTargetCps(
        &s_left_controller,
        s_status.left_target_cps);

    (void)WheelSpeedControl_SetTargetCps(
        &s_right_controller,
        s_status.right_target_cps);

    return true;
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

    left_mm_s = (int64_t)linear_mm_s - turning_mm_s;
    right_mm_s = (int64_t)linear_mm_s + turning_mm_s;

    return Chassis_SetWheelSpeedMmps(
        Chassis_ClampSpeedMmps(left_mm_s),
        Chassis_ClampSpeedMmps(right_mm_s));
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
    s_status.left_pwm = 0;
    s_status.right_pwm = 0;

    WheelSpeedControl_Reset(&s_left_controller);
    WheelSpeedControl_Reset(&s_right_controller);

    BSP_Motor_BrakeAll();
}

bool Chassis_Update(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
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
         * 严重超时后先同步并丢弃这段累计增量，避免按错误周期调节。
         */
        (void)BSP_Encoder_Sample(&encoder_sample);
        Chassis_ResetControllers();
        BSP_Motor_BrakeAll();

        s_status.left_pwm = 0;
        s_status.right_pwm = 0;
        s_status.dt_ms = (uint16_t)elapsed_ms;
        s_status.timestamp_ms = now_ms;
        s_status.timing_overrun_count++;

        s_last_control_ms = now_ms;
        return false;
    }

    s_last_control_ms = now_ms;

    if (!BSP_Encoder_Sample(&encoder_sample))
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

    s_status.left_measured_cps =
        WheelSpeedControl_GetMeasuredCps(&s_left_controller);

    s_status.right_measured_cps =
        WheelSpeedControl_GetMeasuredCps(&s_right_controller);

    s_status.left_measured_mm_s =
        Chassis_DeltaToMmps(
            encoder_sample.left_delta,
            (uint16_t)elapsed_ms);

    s_status.right_measured_mm_s =
        Chassis_DeltaToMmps(
            encoder_sample.right_delta,
            (uint16_t)elapsed_ms);

    s_status.left_pwm = left_pwm;
    s_status.right_pwm = right_pwm;
    s_status.left_delta = encoder_sample.left_delta;
    s_status.right_delta = encoder_sample.right_delta;
    s_status.left_total = encoder_sample.left_total;
    s_status.right_total = encoder_sample.right_total;
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
