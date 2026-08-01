#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief TIM9 的 ARR 值。电机指令绝对值范围为 0～8399。
 */
#define BSP_MOTOR_PWM_MAX    ((int16_t)8399)

/**
 * @brief 初始化 TB6612FNG 电机 BSP。
 *
 * 调用前必须完成 MX_GPIO_Init() 和 MX_TIM9_Init()。
 * 初始化完成后，TB6612 保持待机，左右电机 PWM 为 0。
 *
 * @return true  两路 PWM 启动成功。
 * @return false PWM 启动失败，TB6612 保持待机。
 */
bool BSP_Motor_Init(void);

/**
 * @brief 使能或关闭 TB6612FNG。
 *
 * enable=true：先将两路电机置为短路刹车，再拉高 STBY。
 * enable=false：PWM 清零后拉低 STBY，进入高阻待机。
 *
 * @return true  操作成功。
 * @return false BSP 尚未初始化。
 */
bool BSP_Motor_Enable(bool enable);

/**
 * @brief 查询 TB6612FNG 当前是否已使能。
 */
bool BSP_Motor_IsEnabled(void);

/**
 * @brief 设置左电机指令。
 *
 * command > 0：正转，定义为驱动车轮使小车向前。
 * command < 0：反转。
 * command = 0：短路刹车。
 * 超过范围的数值会自动限制到 ±BSP_MOTOR_PWM_MAX。
 *
 * @return true  指令已写入。
 * @return false BSP 未初始化或 TB6612 未使能，指令未执行。
 */
bool BSP_Motor_SetLeft(int16_t command);

/**
 * @brief 设置右电机指令，规则同 BSP_Motor_SetLeft()。
 */
bool BSP_Motor_SetRight(int16_t command);

/**
 * @brief 同时设置左右电机指令。
 */
bool BSP_Motor_Set(int16_t left_command, int16_t right_command);

/**
 * @brief 左电机短路刹车。
 */
void BSP_Motor_BrakeLeft(void);

/**
 * @brief 右电机短路刹车。
 */
void BSP_Motor_BrakeRight(void);

/**
 * @brief 左右电机同时短路刹车。
 */
void BSP_Motor_BrakeAll(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
