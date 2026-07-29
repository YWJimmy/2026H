#ifndef TEST_SERVO_H
#define TEST_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief 初始化 MG90S 舵机安全范围测试和调试串口。
 *
 * 调用前必须完成：
 * - MX_DMA_Init()
 * - MX_USART1_UART_Init()
 * - MX_TIM1_Init()
 *
 * 测试会启用舵机 PWM，并从 1500 us 中位开始。
 */
bool Test_Servo_Init(void);

/**
 * @brief 非阻塞更新舵机测试。
 *
 * 主循环中持续调用。
 * 默认每 2 秒依次输出：
 * 1500 -> 1450 -> 1500 -> 1550 -> 1500 us，
 * 然后循环执行。
 */
void Test_Servo_Update(void);

/**
 * @brief 停止测试并关闭舵机 PWM。
 */
void Test_Servo_Stop(void);

/**
 * @brief 查询舵机测试是否已经初始化成功。
 */
bool Test_Servo_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_SERVO_H */
