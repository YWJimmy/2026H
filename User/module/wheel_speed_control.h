#ifndef WHEEL_SPEED_CONTROL_H
#define WHEEL_SPEED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t kp_q10;
    int32_t ki_q10;

    int16_t pwm_limit;
    int16_t min_drive_pwm;

    uint16_t nominal_period_ms;
} WheelSpeedConfig_t;

typedef struct
{
    WheelSpeedConfig_t config;

    int32_t target_cps;
    int32_t measured_cps;
    int32_t error_cps;

    int32_t integral_q10;
    int16_t output_pwm;

    bool initialized;
} WheelSpeedController_t;

/**
 * @brief 初始化单轮整数定点PI控制器。
 */
bool WheelSpeedControl_Init(WheelSpeedController_t *controller,
                            const WheelSpeedConfig_t *config);

/**
 * @brief 清除目标、积分、测量值和输出。
 */
void WheelSpeedControl_Reset(WheelSpeedController_t *controller);

/**
 * @brief 设置目标速度，单位为编码器计数/秒。
 */
bool WheelSpeedControl_SetTargetCps(WheelSpeedController_t *controller,
                                    int32_t target_cps);

/**
 * @brief 根据本周期编码器增量计算新PWM。
 *
 * @param delta_counts 本周期编码器增量。
 * @param dt_ms 实际采样间隔，单位ms。
 * @return 有符号PWM；目标为0时返回0并清积分。
 */
int16_t WheelSpeedControl_Update(WheelSpeedController_t *controller,
                                 int16_t delta_counts,
                                 uint16_t dt_ms);

int32_t WheelSpeedControl_GetTargetCps(
    const WheelSpeedController_t *controller);

int32_t WheelSpeedControl_GetMeasuredCps(
    const WheelSpeedController_t *controller);

int32_t WheelSpeedControl_GetErrorCps(
    const WheelSpeedController_t *controller);

int16_t WheelSpeedControl_GetOutputPwm(
    const WheelSpeedController_t *controller);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_CONTROL_H */
