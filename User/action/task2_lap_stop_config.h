#ifndef TASK2_LAP_STOP_CONFIG_H
#define TASK2_LAP_STOP_CONFIG_H

#include <stdint.h>

#include "app_config.h"

#define TASK2_PREDECEL_DISTANCE_MM             ((uint32_t)5800U)
#define TASK2_PREDECEL_CENTER_SPEED_MM_S       ((int32_t)220)
#define TASK2_PREDECEL_MIN_SPEED_MM_S          ((int32_t)220)

/* ADC3/4/5 或 ADC4/5/6 为黑；其他通道状态不影响判定。 */
#define TASK2_A_LINE_ADC345_MASK               ((uint8_t)0x1CU)
#define TASK2_A_LINE_ADC456_MASK               ((uint8_t)0x38U)
#define TASK2_A_LINE_WINDOW_FRAMES             ((uint8_t)5U)
#define TASK2_A_LINE_REQUIRED_FRAMES           ((uint8_t)3U)

#define TASK2_SENSOR_FORWARD_OFFSET_MM         \
    ((uint32_t)APP_ROUTE_SENSOR_FORWARD_OFFSET_MM)

/* 编码器位置外环：接近 85 mm 目标时逐步降低巡线基础速度。 */
#define TASK2_POSITION_MAX_SPEED_MM_S           ((int32_t)120)
#define TASK2_POSITION_MIN_SPEED_MM_S           ((int32_t)40)
#define TASK2_POSITION_KP_Q10                   ((int32_t)1536)

#define TASK2_RUN_TIMEOUT_MS                   ((uint32_t)20000U)
#define TASK2_REPORT_PERIOD_MS                 ((uint32_t)200U)

#endif /* TASK2_LAP_STOP_CONFIG_H */
