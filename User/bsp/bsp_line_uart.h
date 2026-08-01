#ifndef BSP_LINE_UART_H
#define BSP_LINE_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BSP_LINE_UART_CHANNEL_COUNT          8U
#define BSP_LINE_UART_POWER_STABLE_MS        100U
#define BSP_LINE_UART_MODE_SETTLE_MS         5U
#define BSP_LINE_UART_QUERY_PERIOD_MS        5U
#define BSP_LINE_UART_RESPONSE_TIMEOUT_MS    4U
#define BSP_LINE_UART_DATA_VALID_MS          20U

typedef enum
{
    BSP_LINE_UART_STATUS_UNINITIALIZED = 0,
    BSP_LINE_UART_STATUS_POWER_WAIT,
    BSP_LINE_UART_STATUS_MODE_SENDING,
    BSP_LINE_UART_STATUS_MODE_SETTLE,
    BSP_LINE_UART_STATUS_READY,
    BSP_LINE_UART_STATUS_WAIT_RESPONSE,
    BSP_LINE_UART_STATUS_OK,
    BSP_LINE_UART_STATUS_TIMEOUT,
    BSP_LINE_UART_STATUS_UART_ERROR,
    BSP_LINE_UART_STATUS_HAL_BUSY
} BspLineUartStatus_t;

typedef struct
{
    /* bit0~bit7 分别对应模块 S1~S8；BSP 不解释黑白极性。 */
    uint8_t raw_state;

    uint32_t sequence;
    uint32_t timestamp_ms;

    uint32_t query_count;
    uint32_t response_count;
    uint32_t timeout_count;
    uint32_t uart_error_count;
    uint32_t hal_busy_count;
    uint32_t last_uart_error;

    BspLineUartStatus_t status;

    bool initialized;
    bool mode_command_sent;
    bool data_valid;
} BspLineUartSnapshot_t;

bool BSP_LineUart_Init(void);
void BSP_LineUart_Process(void);
void BSP_LineUart_Stop(void);

bool BSP_LineUart_IsInitialized(void);
bool BSP_LineUart_IsReady(void);
bool BSP_LineUart_IsDataValid(void);
bool BSP_LineUart_GetSnapshot(BspLineUartSnapshot_t *snapshot);

const char *BSP_LineUart_StatusName(BspLineUartStatus_t status);

void BSP_LineUart_TxCpltCallback(UART_HandleTypeDef *huart);
void BSP_LineUart_RxCpltCallback(UART_HandleTypeDef *huart);
void BSP_LineUart_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LINE_UART_H */
