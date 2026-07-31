#ifndef TASK2_LAP_STOP_CONFIG_H
#define TASK2_LAP_STOP_CONFIG_H

#include <stdint.h>

#include "app_config.h"

#define TASK2_PREDECEL_DISTANCE_MM             ((uint32_t)5800U)
#define TASK2_PREDECEL_CENTER_SPEED_MM_S       ((int32_t)220)
#define TASK2_PREDECEL_MIN_SPEED_MM_S          ((int32_t)220)

/* 中间两路(bit3/bit4)加左邻(bit2)或右邻(bit5)，连续成立即为停车线。 */
#define TASK2_A_LINE_MIDDLE_MASK               ((uint8_t)0x18U)
#define TASK2_A_LINE_SIDE_MASK                 ((uint8_t)0x24U)
#define TASK2_A_LINE_CONFIRM_MS                ((uint32_t)50U)

#define TASK2_SENSOR_FORWARD_OFFSET_MM         \
    ((uint32_t)APP_ROUTE_SENSOR_FORWARD_OFFSET_MM)
#define TASK2_STOP_DISTANCE_MARGIN_MM          ((uint32_t)0U)

#define TASK2_STOP_NOMINAL_DECEL_MM_S2         ((int32_t)60)
#define TASK2_STOP_MAX_DECEL_MM_S2             ((int32_t)2500)
#define TASK2_STOP_JERK_MULTIPLIER             ((int32_t)20)
#define TASK2_STOP_MIN_JERK_MM_S3              ((int32_t)1200)
#define TASK2_STOP_MAX_JERK_MM_S3              ((int32_t)40000)

#define TASK2_RUN_TIMEOUT_MS                   ((uint32_t)20000U)
#define TASK2_REPORT_PERIOD_MS                 ((uint32_t)200U)

#endif /* TASK2_LAP_STOP_CONFIG_H */
