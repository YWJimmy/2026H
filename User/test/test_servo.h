#ifndef TEST_SERVO_H
#define TEST_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief 初始化舵机两极限往返测试。
 *
 * 调用前必须完成：
 * - MX_DMA_Init()
 * - MX_USART1_UART_Init()
 * - MX_TIM1_Init()
 *
 * 测试使用 BSP 中定义的 1000 us 和 2000 us 两个极限。
 */
bool Test_Servo_Init(void);

/**
 * @brief 非阻塞更新舵机两极限往返测试。
 *
 * 主循环中持续调用。
 */
void Test_Servo_Update(void);

/**
 * @brief 停止测试并关闭舵机 PWM。
 */
void Test_Servo_Stop(void);

/**
 * @brief 查询测试是否已经初始化成功。
 */
bool Test_Servo_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_SERVO_H */
