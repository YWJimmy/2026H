#ifndef BSP_VISION_UART_H
#define BSP_VISION_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BSP_VISION_UART_RING_CAPACITY    256U

typedef struct
{
    uint32_t received_byte_count;
    uint32_t overflow_count;
    uint32_t uart_error_count;
    uint32_t restart_count;
    uint32_t last_uart_error;
    uint16_t queued_byte_count;
    bool initialized;
    bool receiving;
} BspVisionUartStatus_t;

bool BSP_VisionUart_Init(void);
void BSP_VisionUart_Process(void);
void BSP_VisionUart_Stop(void);
bool BSP_VisionUart_PopByte(uint8_t *byte);
bool BSP_VisionUart_GetStatus(BspVisionUartStatus_t *status);

void BSP_VisionUart_RxCpltCallback(UART_HandleTypeDef *huart);
void BSP_VisionUart_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* BSP_VISION_UART_H */
