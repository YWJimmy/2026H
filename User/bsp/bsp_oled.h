#ifndef BSP_OLED_H
#define BSP_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * 参考EDC-2024H中的OLED：
 * - SSD1306
 * - 128x64
 * - 7位地址0x3C，HAL发送地址0x78
 *
 * 2026H使用硬件I2C1：
 * PB6 = SCL
 * PB7 = SDA
 * 400 kHz
 */
#define BSP_OLED_WIDTH                     128U
#define BSP_OLED_HEIGHT                    64U
#define BSP_OLED_PAGE_COUNT                8U
#define BSP_OLED_I2C_ADDRESS_7BIT          0x3CU
#define BSP_OLED_I2C_ADDRESS_HAL           (BSP_OLED_I2C_ADDRESS_7BIT << 1U)

#define BSP_OLED_POWER_STABLE_MS           100U
#define BSP_OLED_RETRY_INTERVAL_MS         1000U
#define BSP_OLED_HEARTBEAT_INTERVAL_MS     1000U
#define BSP_OLED_TRANSFER_TIMEOUT_MS       20U
#define BSP_OLED_TRANSFER_CHUNK            32U

typedef enum
{
    BSP_OLED_STATE_UNINITIALIZED = 0,
    BSP_OLED_STATE_POWER_WAIT,
    BSP_OLED_STATE_INITIALIZING,
    BSP_OLED_STATE_ONLINE,
    BSP_OLED_STATE_RETRY_WAIT
} BspOledState_t;

typedef struct
{
    bool initialized;
    bool online;
    bool transfer_busy;

    BspOledState_t state;

    uint8_t dirty_mask;
    uint8_t active_page;
    uint8_t active_offset;

    uint32_t error_count;
    uint32_t disconnect_count;
    uint32_t reconnect_count;
    uint32_t retry_count;
    uint32_t hal_busy_count;
    uint32_t transfer_count;
    uint32_t last_error;

    uint32_t last_success_ms;
    uint32_t next_retry_ms;
} BspOledStatus_t;

/**
 * @brief 初始化OLED软件状态。
 *
 * 调用前必须完成MX_I2C1_Init()，且I2C1应为400 kHz并启用
 * I2C1事件/错误中断。本函数不阻塞等待OLED上电。
 */
bool BSP_Oled_Init(void);

/**
 * @brief 非阻塞推进初始化、刷新、掉线检测与自动重连。
 *
 * 主循环持续调用。每次最多启动一笔I2C中断传输。
 */
void BSP_Oled_Process(void);

/**
 * @brief 清空128x64帧缓冲区，并标记全部页面待刷新。
 */
void BSP_Oled_Clear(void);

/**
 * @brief 清空一个8像素高页面。
 */
void BSP_Oled_ClearPage(uint8_t page);

/**
 * @brief 显示一个5x7 ASCII字符。
 *
 * 支持0-9、A-Z、空格、冒号、减号、点、问号、
 * 大于号、加号、斜杠和等号。小写字母自动转大写。
 */
void BSP_Oled_DrawChar(uint8_t x, uint8_t page, char character);

/**
 * @brief 显示ASCII字符串。
 */
void BSP_Oled_DrawString(
    uint8_t x,
    uint8_t page,
    const char *text);

/**
 * @brief 显示有符号32位整数。
 */
void BSP_Oled_DrawI32(
    uint8_t x,
    uint8_t page,
    int32_t value);

/**
 * @brief 显示无符号32位整数。
 */
void BSP_Oled_DrawU32(
    uint8_t x,
    uint8_t page,
    uint32_t value);

/**
 * @brief Fill or clear a clipped pixel rectangle in the framebuffer.
 */
void BSP_Oled_FillRect(
    uint8_t x,
    uint8_t y,
    uint8_t width,
    uint8_t height,
    bool on);

/**
 * @brief 将指定页面重新标记为待刷新。
 */
void BSP_Oled_MarkPagesDirty(
    uint8_t first_page,
    uint8_t page_count);

bool BSP_Oled_IsInitialized(void);
bool BSP_Oled_IsOnline(void);
bool BSP_Oled_GetStatus(BspOledStatus_t *status);

/**
 * @brief 要求立即进入掉线恢复流程。
 */
void BSP_Oled_ForceReconnect(void);

const char *BSP_Oled_StateName(BspOledState_t state);

/* HAL I2C全局回调转发接口。 */
void BSP_Oled_TxCpltCallback(I2C_HandleTypeDef *hi2c);
void BSP_Oled_ErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* BSP_OLED_H */
