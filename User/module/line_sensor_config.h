#ifndef LINE_SENSOR_CONFIG_H
#define LINE_SENSOR_CONFIG_H

#include <stdint.h>

#define LINE_SENSOR_BACKEND_ADC8             1U
#define LINE_SENSOR_BACKEND_GPIO8            2U

#define LINE_SENSOR_BACKEND                  LINE_SENSOR_BACKEND_ADC8

/* logical index 0 is leftmost; logical index 7 is rightmost. */
#define LINE_SENSOR_CHANNEL_MAP_INIT          \
{                                             \
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U          \
}

/* Initial estimates only. Replace them with values measured on the car. */
#define LINE_SENSOR_WHITE_RAW_INIT            \
{                                             \
    2700U, 2700U, 2700U, 2700U,              \
    2700U, 2700U, 2700U, 2700U               \
}

#define LINE_SENSOR_BLACK_RAW_INIT            \
{                                             \
    3970U, 3970U, 3970U, 3970U,              \
    3970U, 3970U, 3970U, 3970U               \
}

#define LINE_SENSOR_BLACK_ON_THRESHOLD       600U
#define LINE_SENSOR_WHITE_ON_THRESHOLD       400U
#define LINE_SENSOR_MIN_CALIBRATION_SPAN       16U

#endif /* LINE_SENSOR_CONFIG_H */
