#ifndef BSP_DEBUG_UART_H
#define BSP_DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/**
 * @brief 初始化调试串口 DMA 发送模块。
 *
 * 当前固定使用 USART1（PA9/PA10，115200 8N1）。
 * 调用前必须完成 MX_DMA_Init() 和 MX_USART1_UART_Init()，并在
 * CubeMX 中为 USART1_TX 配置 DMA Normal 模式。
 *
 * @return true  USART1 及 TX DMA 配置有效。
 * @return false USART1 未初始化或未关联 TX DMA。
 */
bool BSP_DebugUart_Init(void);

/**
 * @brief 查询调试串口模块是否初始化成功。
 */
bool BSP_DebugUart_IsInitialized(void);

/**
 * @brief 通过 USART1 TX DMA 排队发送一段原始数据。
 *
 * 本函数只复制数据到内部静态队列，不等待串口发送完成。
 * 队列已满时返回 false，并增加丢弃计数。
 */
bool BSP_DebugUart_Write(const uint8_t *data, size_t length);

/**
 * @brief 调试专用、DMA 非阻塞 printf。
 *
 * 本函数不会重定向标准 printf，避免影响其他库。
 * 单条格式化结果最多 BSP_DEBUG_UART_MESSAGE_MAX_LENGTH-1 字节；
 * 超长内容会被截断后排队发送。
 *
 * @return >=0 实际排队的字符数。
 * @return -1 参数或初始化状态错误。
 * @return -2 发送队列已满。
 */
int BSP_Debug_Printf(const char *format, ...);

/**
 * @brief 尝试启动下一笔 DMA 发送。
 *
 * 正常情况下 BSP_DebugUart_Write/BSP_Debug_Printf 会自动启动 DMA。
 * 主循环中周期调用本函数，可在 HAL 暂时返回 BUSY 后自动恢复。
 */
void BSP_DebugUart_Process(void);

/**
 * @brief 将 HAL UART 发送完成回调转发给本模块。
 *
 * 在全局 HAL_UART_TxCpltCallback() 中调用。
 */
void BSP_DebugUart_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief 将 HAL UART 错误回调转发给本模块。
 *
 * 在全局 HAL_UART_ErrorCallback() 中调用。
 */
void BSP_DebugUart_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief 当前是否有一笔 DMA 正在发送。
 */
bool BSP_DebugUart_IsBusy(void);

/**
 * @brief 获取因队列满或 DMA 错误而丢弃的消息数量。
 */
uint32_t BSP_DebugUart_GetDroppedCount(void);

/**
 * @brief 清空尚未发送的排队消息。
 *
 * 当前正在 DMA 发送的消息不会被中止。
 */
void BSP_DebugUart_ClearPending(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DEBUG_UART_H */
