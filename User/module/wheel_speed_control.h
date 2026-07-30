#ifndef WHEEL_SPEED_CONTROL_H
#define WHEEL_SPEED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_SPEED_FILTER_WINDOW_MAX          8U

typedef struct
{
    int32_t kp_q10;
    int32_t ki_q10;

    int16_t pwm_limit;
    int16_t min_drive_pwm;

    uint16_t nominal_period_ms;
    uint8_t measurement_window;
} WheelSpeedConfig_t;

typedef struct
{
    WheelSpeedConfig_t config;

    int32_t target_cps;
    int32_t raw_measured_cps;
    int32_t measured_cps;
    int32_t error_cps;

    int32_t integral_q10;
    int16_t output_pwm;

    int16_t delta_history[WHEEL_SPEED_FILTER_WINDOW_MAX];
    uint16_t dt_history[WHEEL_SPEED_FILTER_WINDOW_MAX];
    int32_t delta_sum;
    uint32_t dt_sum;
    uint8_t history_index;
    uint8_t history_count;

    bool initialized;
} WheelSpeedController_t;

/**
 * @brief 初始化单轮整数定点PI控制器。
 */
bool WheelSpeedControl_Init(WheelSpeedController_t *controller,
                            const WheelSpeedConfig_t *config);

/**
 * @brief 清除目标、积分、测速窗口和输出，保留控制器配置。
 */
void WheelSpeedControl_Reset(WheelSpeedController_t *controller);

/**
 * @brief 设置目标速度，单位为编码器计数/秒。
 */
bool WheelSpeedControl_SetTargetCps(WheelSpeedController_t *controller,
                                    int32_t target_cps);

/**
 * @brief 运行时更新Q10比例、积分系数。
 *
 * 更新后会清除积分和输出，避免参数突变导致PWM跳变。
 */
bool WheelSpeedControl_SetGainsQ10(WheelSpeedController_t *controller,
                                   int32_t kp_q10,
                                   int32_t ki_q10);

/**
 * @brief 根据本周期编码器增量计算新PWM。
 *
 * 控制器每次调用都运行PI；速度反馈使用最近N个周期滑动平均，
 * N由config.measurement_window设置。
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

int32_t WheelSpeedControl_GetRawMeasuredCps(
    const WheelSpeedController_t *controller);

int32_t WheelSpeedControl_GetMeasuredCps(
    const WheelSpeedController_t *controller);

int32_t WheelSpeedControl_GetErrorCps(
    const WheelSpeedController_t *controller);

int32_t WheelSpeedControl_GetIntegralPwm(
    const WheelSpeedController_t *controller);

int16_t WheelSpeedControl_GetOutputPwm(
    const WheelSpeedController_t *controller);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_CONTROL_H */
