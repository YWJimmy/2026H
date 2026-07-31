#ifndef TASK6_LAP_TARGET_CONFIG_H
#define TASK6_LAP_TARGET_CONFIG_H

#include <stdint.h>

#include "task5_lap_hold_config.h"

/* Task 6 captures the first fresh detected ball position after start. */
#define TASK6_TARGET_CAPTURE_TIMEOUT_MS       ((uint32_t)2000U)

/* Chassis behavior must remain identical to Task 5. */
#define TASK6_A_LINE_SEARCH_DISTANCE_MM       TASK5_A_LINE_SEARCH_DISTANCE_MM
#define TASK6_A_LINE_ADC345_MASK              TASK5_A_LINE_ADC345_MASK
#define TASK6_A_LINE_ADC456_MASK              TASK5_A_LINE_ADC456_MASK
#define TASK6_A_LINE_WINDOW_FRAMES            TASK5_A_LINE_WINDOW_FRAMES
#define TASK6_A_LINE_REQUIRED_FRAMES          TASK5_A_LINE_REQUIRED_FRAMES
#define TASK6_POST_LINE_DISTANCE_MM           TASK5_POST_LINE_DISTANCE_MM
#define TASK6_STOP_DECEL_MM_S2                TASK5_STOP_DECEL_MM_S2
#define TASK6_STOP_JERK_MM_S3                 TASK5_STOP_JERK_MM_S3
#define TASK6_LAP_TIMEOUT_MS                  TASK5_LAP_TIMEOUT_MS
#define TASK6_POST_LINE_TIMEOUT_MS            TASK5_POST_LINE_TIMEOUT_MS
#define TASK6_STOP_TIMEOUT_MS                 TASK5_STOP_TIMEOUT_MS
#define TASK6_REPORT_PERIOD_MS                TASK5_REPORT_PERIOD_MS

#endif /* TASK6_LAP_TARGET_CONFIG_H */
