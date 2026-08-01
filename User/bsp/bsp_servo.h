#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief MG90S 舵机允许的逻辑脉宽下限，单位为 us。
 */
#define BSP_SERVO_PULSE_MIN_US       ((uint16_t)500U)

/**
 * @brief MG90S 舵机中位逻辑脉宽，单位为 us。
 */
#define BSP_SERVO_PULSE_CENTER_US    ((uint16_t)1500U)

/**
 * @brief MG90S 舵机允许的逻辑脉宽上限，单位为 us。
 */
#define BSP_SERVO_PULSE_MAX_US       ((uint16_t)2500U)

/**
 * @brief 初始化舵机 BSP。
 *
 * 硬件映射：
 * - TIM1_CH1
 * - PA8 / SERVO_PWM
 * - 50 Hz
 * - 1 us/计数
 *
 * 调用前必须完成 MX_TIM1_Init()。
 * 初始化后只设置内部目标为中位，不启动 PWM 输出。
 *
 * @return true  TIM1 配置符合要求，初始化成功。
 * @return false TIM1 配置不符合要求。
 */
bool BSP_Servo_Init(void);

/**
 * @brief 启动 TIM1_CH1 PWM 输出。
 *
 * 启动时输出当前保存的逻辑脉宽，初始化后的默认值为 1500 us。
 *
 * @return true  已启动，或原本已经启动。
 * @return false BSP 尚未初始化，或 HAL 启动失败。
 */
bool BSP_Servo_Enable(void);

/**
 * @brief 停止 TIM1_CH1 PWM 输出。
 *
 * 停止前将内部目标和 CCR 恢复为 1500 us，
 * 便于下次 Enable() 从中位开始。
 */
void BSP_Servo_Disable(void);

/**
 * @brief 查询舵机 BSP 是否已经初始化。
 */
bool BSP_Servo_IsInitialized(void);

/**
 * @brief 查询舵机 PWM 当前是否正在输出。
 */
bool BSP_Servo_IsEnabled(void);

/**
 * @brief 设置舵机逻辑脉宽，单位为 us。
 *
 * 输入值会被限制到 1500～2500 us。
 * BSP 未使能时只保存目标值，不产生 PWM；
 * BSP 已使能时立即更新 TIM1_CH1 的 CCR。
 *
 * @param pulse_us 目标逻辑脉宽。
 * @return true  已保存或写入。
 * @return false BSP 尚未初始化。
 */
bool BSP_Servo_SetPulseUs(uint16_t pulse_us);

/**
 * @brief 以中位为基准设置整数微秒偏移。
 *
 * 示例：
 * - offset_us = 0    -> 1500 us
 * - offset_us = -50  -> 1450 us
 * - offset_us = +50  -> 1550 us
 *
 * 超出 500～2500 us 的结果会自动限幅。
 *
 * @param offset_us 相对中位的偏移量，单位为 us。
 * @return true  已保存或写入。
 * @return false BSP 尚未初始化。
 */
bool BSP_Servo_SetOffsetUs(int16_t offset_us);

/**
 * @brief 获取当前保存的逻辑脉宽，单位为 us。
 *
 * 返回的是方向修正前的逻辑命令值。
 */
uint16_t BSP_Servo_GetPulseUs(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
