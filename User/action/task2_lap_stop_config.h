#ifndef TASK2_LAP_STOP_CONFIG_H
#define TASK2_LAP_STOP_CONFIG_H

#include <stdint.h>

#include "app_config.h"

/* Task 2专用巡线速度，不改变任务4/5/6的全局巡线参数。 */
#define TASK2_CRUISE_CENTER_SPEED_MM_S         ((int32_t)390)
#define TASK2_CRUISE_MIN_SPEED_MM_S            ((int32_t)320)

/* 5.6 m后才进入停止线搜索，缩短低速运行区间。 */
#define TASK2_PREDECEL_DISTANCE_MM             ((uint32_t)5600U)
#define TASK2_PREDECEL_CENTER_SPEED_MM_S       ((int32_t)260)
#define TASK2_PREDECEL_MIN_SPEED_MM_S          ((int32_t)260)

/* 兼容保留的停止线掩码；当前帧判定采用任意3路及以上为黑。 */
#define TASK2_A_LINE_ADC345_MASK               ((uint8_t)0x1CU)
#define TASK2_A_LINE_ADC456_MASK               ((uint8_t)0x38U)
#define TASK2_A_LINE_WINDOW_FRAMES             ((uint8_t)5U)
#define TASK2_A_LINE_REQUIRED_FRAMES           ((uint8_t)3U)

#define TASK2_SENSOR_FORWARD_OFFSET_MM         \
    ((uint32_t)80U)

/* 编码器位置外环：识别停止线后按传感器前置距离逐步减速。 */
#define TASK2_POSITION_MAX_SPEED_MM_S           ((int32_t)120)
#define TASK2_POSITION_MIN_SPEED_MM_S           ((int32_t)40)
#define TASK2_POSITION_KP_Q10                   ((int32_t)1536)

#define TASK2_RUN_TIMEOUT_MS                   ((uint32_t)20000U)
#define TASK2_REPORT_PERIOD_MS                 ((uint32_t)200U)

#endif /* TASK2_LAP_STOP_CONFIG_H */
