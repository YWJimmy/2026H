#include "wheel_speed_control.h"

#include <limits.h>
#include <stddef.h>

#define WHEEL_SPEED_Q10_SCALE    ((int32_t)1024)

static int32_t WheelSpeed_ClampInt32(int64_t value,
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

static int16_t WheelSpeed_ClampPwm(int64_t value, int16_t limit)
{
    if (value > (int64_t)limit)
    {
        return limit;
    }

    if (value < -(int64_t)limit)
    {
        return (int16_t)(-limit);
    }

    return (int16_t)value;
}

static int16_t WheelSpeed_ApplyMinimumDrive(
    int16_t pwm,
    int32_t target_cps,
    int16_t minimum_drive_pwm)
{
    int32_t magnitude;

    if ((target_cps == 0) || (minimum_drive_pwm <= 0))
    {
        return pwm;
    }

    /*
     * 只对与目标同方向的驱动量做死区补偿。
     * 若PI因超速给出反向修正，不应把它强制改回目标方向。
     */
    if ((pwm == 0) ||
        ((pwm > 0) && (target_cps < 0)) ||
        ((pwm < 0) && (target_cps > 0)))
    {
        return pwm;
    }

    magnitude = (pwm < 0) ? -(int32_t)pwm : (int32_t)pwm;

    if (magnitude >= (int32_t)minimum_drive_pwm)
    {
        return pwm;
    }

    return (pwm > 0) ?
        minimum_drive_pwm :
        (int16_t)(-minimum_drive_pwm);
}

bool WheelSpeedControl_Init(WheelSpeedController_t *controller,
                            const WheelSpeedConfig_t *config)
{
    if ((controller == NULL) ||
        (config == NULL) ||
        (config->kp_q10 < 0) ||
        (config->ki_q10 < 0) ||
        (config->pwm_limit <= 0) ||
        (config->min_drive_pwm < 0) ||
        (config->min_drive_pwm > config->pwm_limit) ||
        (config->nominal_period_ms == 0U))
    {
        return false;
    }

    controller->config = *config;
    controller->target_cps = 0;
    controller->measured_cps = 0;
    controller->error_cps = 0;
    controller->integral_q10 = 0;
    controller->output_pwm = 0;
    controller->initialized = true;

    return true;
}

void WheelSpeedControl_Reset(WheelSpeedController_t *controller)
{
    if ((controller == NULL) || (!controller->initialized))
    {
        return;
    }

    controller->target_cps = 0;
    controller->measured_cps = 0;
    controller->error_cps = 0;
    controller->integral_q10 = 0;
    controller->output_pwm = 0;
}

bool WheelSpeedControl_SetTargetCps(WheelSpeedController_t *controller,
                                    int32_t target_cps)
{
    bool direction_changed;

    if ((controller == NULL) || (!controller->initialized))
    {
        return false;
    }

    direction_changed =
        ((controller->target_cps > 0) && (target_cps < 0)) ||
        ((controller->target_cps < 0) && (target_cps > 0));

    if ((target_cps == 0) || direction_changed)
    {
        controller->integral_q10 = 0;
        controller->output_pwm = 0;
    }

    controller->target_cps = target_cps;
    return true;
}

int16_t WheelSpeedControl_Update(WheelSpeedController_t *controller,
                                 int16_t delta_counts,
                                 uint16_t dt_ms)
{
    int64_t p_q10;
    int64_t integral_step_q10;
    int64_t candidate_integral_q10;
    int64_t sum_q10;
    int64_t raw_pwm;
    int32_t integral_limit_q10;
    bool saturating_high;
    bool saturating_low;

    if ((controller == NULL) ||
        (!controller->initialized) ||
        (dt_ms == 0U))
    {
        return 0;
    }

    controller->measured_cps =
        (int32_t)(((int64_t)delta_counts * 1000LL) /
                  (int64_t)dt_ms);

    controller->error_cps =
        controller->target_cps - controller->measured_cps;

    if (controller->target_cps == 0)
    {
        controller->integral_q10 = 0;
        controller->output_pwm = 0;
        return 0;
    }

    p_q10 =
        (int64_t)controller->config.kp_q10 *
        (int64_t)controller->error_cps;

    integral_step_q10 =
        ((int64_t)controller->config.ki_q10 *
         (int64_t)controller->error_cps *
         (int64_t)dt_ms) /
        (int64_t)controller->config.nominal_period_ms;

    integral_limit_q10 =
        (int32_t)controller->config.pwm_limit *
        WHEEL_SPEED_Q10_SCALE;

    candidate_integral_q10 =
        (int64_t)controller->integral_q10 +
        integral_step_q10;

    candidate_integral_q10 = WheelSpeed_ClampInt32(
        candidate_integral_q10,
        -integral_limit_q10,
        integral_limit_q10);

    sum_q10 = p_q10 + candidate_integral_q10;
    raw_pwm = sum_q10 / WHEEL_SPEED_Q10_SCALE;

    saturating_high =
        (raw_pwm > (int64_t)controller->config.pwm_limit) &&
        (controller->error_cps > 0);

    saturating_low =
        (raw_pwm < -(int64_t)controller->config.pwm_limit) &&
        (controller->error_cps < 0);

    /*
     * 条件积分抗饱和：
     * 只有当误差不会把输出继续推向同方向饱和时，才接收新积分值。
     */
    if ((!saturating_high) && (!saturating_low))
    {
        controller->integral_q10 =
            (int32_t)candidate_integral_q10;
    }

    sum_q10 =
        p_q10 + (int64_t)controller->integral_q10;

    raw_pwm = sum_q10 / WHEEL_SPEED_Q10_SCALE;

    controller->output_pwm = WheelSpeed_ClampPwm(
        raw_pwm,
        controller->config.pwm_limit);

    /*
     * 输出方向与目标方向保持一致。
     * “允许反转”表示目标可为负，不表示正向目标超速时主动反打。
     */
    if (((controller->target_cps > 0) &&
         (controller->output_pwm < 0)) ||
        ((controller->target_cps < 0) &&
         (controller->output_pwm > 0)))
    {
        controller->output_pwm = 0;
    }

    controller->output_pwm = WheelSpeed_ApplyMinimumDrive(
        controller->output_pwm,
        controller->target_cps,
        controller->config.min_drive_pwm);

    return controller->output_pwm;
}

int32_t WheelSpeedControl_GetTargetCps(
    const WheelSpeedController_t *controller)
{
    if ((controller == NULL) || (!controller->initialized))
    {
        return 0;
    }

    return controller->target_cps;
}

int32_t WheelSpeedControl_GetMeasuredCps(
    const WheelSpeedController_t *controller)
{
    if ((controller == NULL) || (!controller->initialized))
    {
        return 0;
    }

    return controller->measured_cps;
}

int32_t WheelSpeedControl_GetErrorCps(
    const WheelSpeedController_t *controller)
{
    if ((controller == NULL) || (!controller->initialized))
    {
        return 0;
    }

    return controller->error_cps;
}

int16_t WheelSpeedControl_GetOutputPwm(
    const WheelSpeedController_t *controller)
{
    if ((controller == NULL) || (!controller->initialized))
    {
        return 0;
    }

    return controller->output_pwm;
}
