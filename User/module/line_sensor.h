#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT               8U
#define LINE_SENSOR_STRENGTH_MAX         1000U

typedef struct
{
    uint16_t raw[LINE_SENSOR_COUNT];
    uint16_t strength[LINE_SENSOR_COUNT];
    uint8_t black_mask;
    uint8_t valid_mask;
    uint32_t sequence;
    uint32_t timestamp_ms;
} LineSensorFrame_t;

bool LineSensor_Init(void);
bool LineSensor_Start(void);
bool LineSensor_Stop(void);
bool LineSensor_Update(void);

bool LineSensor_IsInitialized(void);
bool LineSensor_IsRunning(void);
bool LineSensor_GetFrame(LineSensorFrame_t *frame);

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
