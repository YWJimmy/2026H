#ifndef TASK6_LAP_TARGET_CONFIG_H
#define TASK6_LAP_TARGET_CONFIG_H

#include <stdint.h>
#include "task5_lap_hold_config.h"

#define TASK6_TARGET_CAPTURE_TIMEOUT_MS       ((uint32_t)2000U)

/*
 * Task 6 startup transient: establish a moderate physical preload while the
 * chassis is stationary, then use a time-only speed ramp. Ball feedback must
 * not change chassis speed because that couples chassis inertia back into the
 * ball loop and caused the V7 limit cycle.
 */
#define TASK6_START_PRELOAD_MIN_MS            ((uint32_t)300U)
#define TASK6_START_PRELOAD_TIMEOUT_MS        ((uint32_t)700U)
#define TASK6_START_RAMP_MS                   ((uint32_t)10000U)
#define TASK6_START_RAMP_MAX_MS               ((uint32_t)12000U)
#define TASK6_START_CENTER_SPEED_MM_S         ((int32_t)60)
#define TASK6_START_MIN_SPEED_MM_S            ((int32_t)50)
#define TASK6_START_EXIT_MAX_ERROR_MM         ((int32_t)5)
#define TASK6_START_EXIT_MAX_VEL_PX_S         ((int32_t)80)

/*
 * V11 constant cruise: V9 showed stable one-way chassis steering but the tray
 * servo repeatedly crossed its full working range in bends.  Center and
 * minimum speed are deliberately equal, so line error only changes wheel
 * differential and cannot command a longitudinal deceleration on bend entry
 * or acceleration on bend exit.
 */
#define TASK6_CRUISE_CENTER_SPEED_MM_S        ((int32_t)260)
#define TASK6_CRUISE_MIN_SPEED_MM_S           ((int32_t)260)

/* Hold the curve-entry command across one short line-sensor dropout. */
#define TASK6_LOST_COMMAND_HOLD_MS             ((uint32_t)250U)

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
#define TASK6_STOP_APPROACH_CENTER_SPEED_MM_S TASK6_CRUISE_CENTER_SPEED_MM_S
#define TASK6_STOP_APPROACH_MIN_SPEED_MM_S    TASK6_CRUISE_MIN_SPEED_MM_S
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
