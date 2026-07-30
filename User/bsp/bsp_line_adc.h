#ifndef BSP_LINE_ADC_H
#define BSP_LINE_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BSP_LINE_ADC_CHANNEL_COUNT       8U
#define BSP_LINE_ADC_STABILIZE_MS        5U

typedef struct
{
    uint16_t raw[BSP_LINE_ADC_CHANNEL_COUNT];
    uint32_t sequence;
    uint32_t timestamp_ms;
} BspLineAdcSnapshot_t;

bool BSP_LineAdc_Init(void);
bool BSP_LineAdc_Start(void);
bool BSP_LineAdc_Stop(void);

bool BSP_LineAdc_IsInitialized(void);
bool BSP_LineAdc_IsRunning(void);
bool BSP_LineAdc_IsDataValid(void);

bool BSP_LineAdc_GetRaw(uint16_t raw[BSP_LINE_ADC_CHANNEL_COUNT]);
bool BSP_LineAdc_GetChannel(uint8_t channel, uint16_t *raw);
bool BSP_LineAdc_GetSnapshot(BspLineAdcSnapshot_t *snapshot);

uint32_t BSP_LineAdc_GetSequence(void);
uint32_t BSP_LineAdc_GetErrorCount(void);

void BSP_LineAdc_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void BSP_LineAdc_ErrorCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LINE_ADC_H */
