#ifndef TASK6_LAP_TARGET_CONFIG_H
#define TASK6_LAP_TARGET_CONFIG_H

#include <stdint.h>
#include "task5_lap_hold_config.h"

#define TASK6_TARGET_CAPTURE_TIMEOUT_MS       ((uint32_t)2000U)

/* Task 6 startup transient: capture, hold still, then ramp from low speed. */
#define TASK6_START_HOLD_MS                   ((uint32_t)300U)
#define TASK6_START_RAMP_MS                   ((uint32_t)1400U)
#define TASK6_START_CENTER_SPEED_MM_S         ((int32_t)120)
#define TASK6_START_MIN_SPEED_MM_S            ((int32_t)100)

#define TASK6_A_LINE_SEARCH_DISTANCE_MM       TASK5_A_LINE_SEARCH_DISTANCE_MM
#define TASK6_A_LINE_ADC345_MASK              TASK5_A_LINE_ADC345_MASK
#define TASK6_A_LINE_ADC456_MASK              TASK5_A_LINE_ADC456_MASK
#define TASK6_A_LINE_WINDOW_FRAMES            TASK5_A_LINE_WINDOW_FRAMES
#define TASK6_A_LINE_REQUIRED_FRAMES          TASK5_A_LINE_REQUIRED_FRAMES
#define TASK6_POST_LINE_DISTANCE_MM           TASK5_POST_LINE_DISTANCE_MM

/*
 * Task 6 parking transient. Begin decelerating immediately after A, then
 * lower the speed again for the final section before the chassis stop profile.
 */
#define TASK6_STOP_APPROACH_CENTER_SPEED_MM_S ((int32_t)280)
#define TASK6_STOP_APPROACH_MIN_SPEED_MM_S    ((int32_t)220)
#define TASK6_STOP_FINAL_STAGE_DISTANCE_MM    ((uint32_t)160U)
#define TASK6_STOP_FINAL_CENTER_SPEED_MM_S    ((int32_t)180)
#define TASK6_STOP_FINAL_MIN_SPEED_MM_S       ((int32_t)140)
#define TASK6_STOP_DECEL_MM_S2                ((int32_t)180)
#define TASK6_STOP_JERK_MM_S3                 ((int32_t)800)
#define TASK6_STOP_SETTLE_MS                  ((uint32_t)400U)
#define TASK6_LAP_TIMEOUT_MS                  TASK5_LAP_TIMEOUT_MS
#define TASK6_POST_LINE_TIMEOUT_MS            TASK5_POST_LINE_TIMEOUT_MS
#define TASK6_STOP_TIMEOUT_MS                 TASK5_STOP_TIMEOUT_MS
#define TASK6_REPORT_PERIOD_MS                TASK5_REPORT_PERIOD_MS

#endif /* TASK6_LAP_TARGET_CONFIG_H */
