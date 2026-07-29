#ifndef TEST_ENCODER_H
#define TEST_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief 初始化编码器计数测试及调试串口 DMA 输出。
 *
 * 调用前必须完成 MX_DMA_Init()、MX_USART1_UART_Init()、
 * MX_TIM3_Init() 和 MX_TIM4_Init()。
 */
bool Test_Encoder_Init(void);

/**
 * @brief 非阻塞更新编码器测试。
 *
 * 主循环中持续调用；默认每 100 ms 输出一帧数据。
 */
void Test_Encoder_Update(void);

/**
 * @brief 复位左右编码器累计计数。
 */
void Test_Encoder_Reset(void);

/**
 * @brief 查询测试是否已初始化成功。
 */
bool Test_Encoder_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_ENCODER_H */
