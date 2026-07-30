#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool initialized;
    bool enabled;
    bool encoder_sample_valid;
    bool left_output_saturated;
    bool right_output_saturated;

    int32_t left_target_mm_s;
    int32_t right_target_mm_s;
    int32_t left_measured_mm_s;
    int32_t right_measured_mm_s;

    int32_t left_target_cps;
    int32_t right_target_cps;
    int32_t left_raw_measured_cps;
    int32_t right_raw_measured_cps;
    int32_t left_measured_cps;
    int32_t right_measured_cps;
    int32_t left_error_cps;
    int32_t right_error_cps;

    int16_t left_feedforward_pwm;
    int16_t right_feedforward_pwm;
    int32_t left_proportional_pwm;
    int32_t right_proportional_pwm;
    int32_t left_integral_pwm;
    int32_t right_integral_pwm;
    int16_t left_pwm;
    int16_t right_pwm;

    int16_t left_delta;
    int16_t right_delta;
    int32_t left_total;
    int32_t right_total;

    uint16_t dt_ms;
    uint32_t control_sequence;
    uint32_t timestamp_ms;
    uint32_t timing_overrun_count;
    uint32_t left_encoder_reject_count;
    uint32_t right_encoder_reject_count;
} ChassisStatus_t;

/**
 * @brief 初始化电机、编码器和左右轮“前馈+PI”控制器。
 *
 * 初始化后TB6612保持待机，不会驱动电机。
 */
bool Chassis_Init(void);

/**
 * @brief 使能或关闭底盘。
 *
 * 使能时重置编码器基准和控制器状态；
 * 关闭时立即短路刹车并拉低TB6612 STBY。
 */
bool Chassis_Enable(bool enable);

bool Chassis_IsInitialized(void);
bool Chassis_IsEnabled(void);

/**
 * @brief 将mm/s转换为编码器count/s，不使用浮点数。
 */
int32_t Chassis_MmpsToCps(int32_t speed_mm_s);

/**
 * @brief 将编码器count/s转换为mm/s，不使用浮点数。
 */
int32_t Chassis_CpsToMmps(int32_t speed_cps);

/**
 * @brief 直接设置左右轮目标速度，单位mm/s。
 */
bool Chassis_SetWheelSpeedMmps(int32_t left_mm_s,
                               int32_t right_mm_s);

/**
 * @brief 直接设置左右轮目标速度，单位count/s。
 */
bool Chassis_SetWheelSpeedCps(int32_t left_cps,
                              int32_t right_cps);

/**
 * @brief 设置底盘线速度和角速度。
 *
 * @param linear_mm_s 车体中心线速度，正值前进。
 * @param angular_mrad_s 角速度，正值左转（逆时针）。
 */
bool Chassis_SetVelocity(int32_t linear_mm_s,
                         int32_t angular_mrad_s);

/**
 * @brief 运行时更新左右轮Q10 PI参数。
 *
 * 更新后左右积分和PWM输出清零。
 */
bool Chassis_SetWheelPiGainsQ10(int32_t left_kp_q10,
                                int32_t left_ki_q10,
                                int32_t right_kp_q10,
                                int32_t right_ki_q10);

/**
 * @brief 运行时更新左右轮前馈参数。
 */
bool Chassis_SetWheelFeedforwardQ10(
    int32_t left_gain_q10,
    int16_t left_static_pwm,
    int32_t right_gain_q10,
    int16_t right_static_pwm);

/**
 * @brief 目标清零、清控制器并立即短路刹车。
 */
void Chassis_Stop(void);

/**
 * @brief 非阻塞更新底盘闭环。
 *
 * 主循环中持续调用，标称每5 ms执行一次控制。
 *
 * @return true 本次执行了一个有效控制周期。
 * @return false 尚未到周期、未使能、严重超时或编码器样本被拒绝。
 */
bool Chassis_Update(void);

/**
 * @brief 获取最近一次底盘状态快照。
 */
bool Chassis_GetStatus(ChassisStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
