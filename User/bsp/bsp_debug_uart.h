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
 * @brief 初始化 USART1 DMA 与 USART2 蓝牙镜像发送模块。
 *
 * Debug 使用 USART1（PA9/PA10），蓝牙镜像使用 USART2（PA2/PA3），
 * 两者均为 115200 8N1。调用前必须完成 MX_DMA_Init()、USART1 和
 * USART2 初始化，并为 USART1_TX 配置 DMA Normal 模式。
 *
 * @return true  USART1/TX DMA 与 USART2 配置有效。
 * @return false 任一日志串口配置无效。
 */
bool BSP_DebugUart_Init(void);

/**
 * @brief 查询调试串口模块是否初始化成功。
 */
bool BSP_DebugUart_IsInitialized(void);

/**
 * @brief 将原始数据同时排入 USART1 DMA 和 USART2 蓝牙发送队列。
 *
 * 本函数只复制数据到内部静态队列，不等待串口发送完成。
 * 队列已满时返回 false，并增加丢弃计数。
 * 超过单条消息容量时会截断并增加截断计数。
 */
bool BSP_DebugUart_Write(const uint8_t *data, size_t length);

/**
 * @brief 调试专用、DMA 非阻塞 printf。
 *
 * 本函数不会重定向标准 printf，避免影响其他库。
 * 超长内容会被截断为带“...\r\n”结尾的独立消息，
 * 防止下一条日志与本条日志粘连。
 *
 * @return >=0 实际排队的字符数。
 * @return -1 参数或初始化状态错误。
 * @return -2 发送队列已满。
 */
int BSP_Debug_Printf(const char *format, ...);

/**
 * @brief 尝试启动下一笔 USART1 DMA 和 USART2 中断发送。
 */
void BSP_DebugUart_Process(void);

/**
 * @brief 将 HAL UART 发送完成回调转发给本模块。
 */
void BSP_DebugUart_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief 将 HAL UART 错误回调转发给本模块。
 */
void BSP_DebugUart_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief 当前任一日志串口是否正在发送。
 */
bool BSP_DebugUart_IsBusy(void);

/**
 * @brief 获取因队列满或 DMA 错误而丢弃的消息数量。
 */
uint32_t BSP_DebugUart_GetDroppedCount(void);

/**
 * @brief 获取因单条消息过长而发生的截断次数。
 */
uint32_t BSP_DebugUart_GetTruncatedCount(void);

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
