#ifndef TEST_VISION_UART_H
#define TEST_VISION_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief 初始化视觉 UART 接收测试。
 *
 * 调用前必须完成：
 * - MX_DMA_Init()
 * - MX_USART1_UART_Init()
 * - MX_USART6_UART_Init()
 */
bool Test_VisionUart_Init(void);

/**
 * @brief 主循环中周期调用，接收并转发 K230D 数据。
 */
void Test_VisionUart_Update(void);

/**
 * @brief 停止测试。
 */
void Test_VisionUart_Stop(void);

/**
 * @brief 查询测试是否已初始化。
 */
bool Test_VisionUart_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_VISION_UART_H */
