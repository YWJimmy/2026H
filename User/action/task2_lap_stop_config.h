#ifndef TASK2_LAP_STOP_CONFIG_H
#define TASK2_LAP_STOP_CONFIG_H

#include <stdint.h>

#include "app_config.h"

#define TASK2_PREDECEL_DISTANCE_MM             ((uint32_t)5800U)
#define TASK2_PREDECEL_CENTER_SPEED_MM_S       ((int32_t)220)
#define TASK2_PREDECEL_MIN_SPEED_MM_S          ((int32_t)220)

/* 5.8 m 开始减速并同步搜索；bit2～bit5 必须连续为黑一段时间。 */
#define TASK2_A_LINE_CENTER_MASK               ((uint8_t)0x3CU)
#define TASK2_A_LINE_CONFIRM_MS                ((uint32_t)50U)

#define TASK2_SENSOR_FORWARD_OFFSET_MM         \
    ((uint32_t)APP_ROUTE_SENSOR_FORWARD_OFFSET_MM)
#define TASK2_STOP_DISTANCE_MARGIN_MM          ((uint32_t)0U)

#define TASK2_STOP_NOMINAL_DECEL_MM_S2         ((int32_t)60)
#define TASK2_STOP_MAX_DECEL_MM_S2             ((int32_t)2500)
#define TASK2_STOP_JERK_MULTIPLIER             ((int32_t)20)
#define TASK2_STOP_MIN_JERK_MM_S3              ((int32_t)1200)
#define TASK2_STOP_MAX_JERK_MM_S3              ((int32_t)40000)

/* 20 s 是成绩目标；30 s 是未找到停车线时的安全故障上限。 */
#define TASK2_RUN_TIMEOUT_MS                   ((uint32_t)30000U)
#define TASK2_REPORT_PERIOD_MS                 ((uint32_t)200U)

#endif /* TASK2_LAP_STOP_CONFIG_H */
