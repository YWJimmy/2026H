#ifndef BSP_VISION_UART_H
#define BSP_VISION_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/**
 * @brief 解析后的小钢球检测结果。
 *
 * 坐标系为 K230D 输出的 1280×960 像素坐标，
 * 未经任何变换。
 */
typedef struct
{
    bool     has_target;   ///< true：当前有目标
    uint16_t x1;           ///< 检测框左上角 X（1280 坐标）
    uint16_t y1;           ///< 检测框左上角 Y（960  坐标）
    uint16_t x2;           ///< 检测框右下角 X
    uint16_t y2;           ///< 检测框右下角 Y
    uint16_t cx;           ///< 检测框中心 X
    uint16_t cy;           ///< 检测框中心 Y
    uint16_t score_milli;  ///< 观测置信度 ×1000；短时预测帧为 0
    uint32_t timestamp_ms; ///< 接收到该帧时的 HAL_GetTick()
} BspVisionDetection_t;

/**
 * @brief 初始化视觉模块 UART 接收。
 *
 * 使用 USART6（PG9=RX，PG14=TX，115200 8N1），中断逐字节接收。
 * 调用前必须完成 MX_USART6_UART_Init()。
 *
 * @return true  USART6 已配置，启动接收成功。
 * @return false USART6 未配置或启动失败。
 */
bool BSP_VisionUart_Init(void);

/**
 * @brief 查询视觉 UART 模块是否已初始化。
 */
bool BSP_VisionUart_IsInitialized(void);

/**
 * @brief 主循环中周期调用，处理超时恢复等。
 */
void BSP_VisionUart_Process(void);

/**
 * @brief 自上次 GetDetection 后是否有新的检测帧到达。
 */
bool BSP_VisionUart_HasNewDetection(void);

/**
 * @brief 读取最近一次解析成功的检测结果。
 *
 * 调用后 new detection 标志会被清除。
 * 若尚未收到任何有效帧，返回 has_target==false 的全零结果。
 */
BspVisionDetection_t BSP_VisionUart_GetDetection(void);

/**
 * @brief 将 HAL UART 接收完成回调转发给本模块。
 *
 * 在全局 HAL_UART_RxCpltCallback() 中调用。
 */
void BSP_VisionUart_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief 将 HAL UART 错误回调转发给本模块。
 *
 * 在全局 HAL_UART_ErrorCallback() 中调用。
 */
void BSP_VisionUart_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief 获取已成功接收的帧总数。
 */
uint32_t BSP_VisionUart_GetFrameCount(void);

/**
 * @brief 获取因格式错误丢弃的帧总数。
 */
uint32_t BSP_VisionUart_GetErrorCount(void);

/* ---- Stubs for module-layer compatibility ---- */

typedef struct {
    uint32_t received_byte_count;
    uint32_t overflow_count;
    uint32_t uart_error_count;
    uint32_t restart_count;
    uint32_t last_uart_error;
    uint16_t queued_byte_count;
} BspVisionUartStatus_t;

void BSP_VisionUart_Stop(void);
bool BSP_VisionUart_PopByte(uint8_t *byte);
bool BSP_VisionUart_GetStatus(BspVisionUartStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* BSP_VISION_UART_H */
