#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT               8U
#define LINE_SENSOR_STRENGTH_MAX        1000U

typedef struct
{
    /* ADC 后端为采样值；UART 数字后端为 0/1。 */
    uint16_t raw[LINE_SENSOR_COUNT];

    /* 统一黑度：0=白，1000=黑。 */
    uint16_t strength[LINE_SENSOR_COUNT];

    /* bit0=逻辑最左，bit7=逻辑最右；1=黑线。 */
    uint8_t black_mask;
    uint8_t valid_mask;

    uint32_t sequence;
    uint32_t timestamp_ms;
} LineSensorFrame_t;

bool LineSensor_Init(void);
bool LineSensor_Start(void);
bool LineSensor_Stop(void);
bool LineSensor_Update(void);
void LineSensor_DiscardFrame(void);

bool LineSensor_IsInitialized(void);
bool LineSensor_IsRunning(void);
bool LineSensor_IsBackendConfigConfirmed(void);
const char *LineSensor_GetBackendName(void);

bool LineSensor_GetFrame(LineSensorFrame_t *frame);

/* 仅 ADC 后端支持运行时标定；UART 数字后端返回 false。 */
bool LineSensor_SetCalibration(uint8_t logical_channel,
                               uint16_t white_raw,
                               uint16_t black_raw);
bool LineSensor_GetCalibration(uint8_t logical_channel,
                               uint16_t *white_raw,
                               uint16_t *black_raw);
void LineSensor_ResetCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_SENSOR_H */
