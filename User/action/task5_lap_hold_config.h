#ifndef TASK5_LAP_HOLD_CONFIG_H
#define TASK5_LAP_HOLD_CONFIG_H

#include <stdint.h>

#include "ball_balance_control_config.h"
#include "task2_lap_stop_config.h"

/* Keep Task 5 A-line detection identical to Task 2. */
#define TASK5_A_LINE_SEARCH_DISTANCE_MM       TASK2_PREDECEL_DISTANCE_MM
#define TASK5_A_LINE_ADC345_MASK              TASK2_A_LINE_ADC345_MASK
#define TASK5_A_LINE_ADC456_MASK              TASK2_A_LINE_ADC456_MASK
#define TASK5_A_LINE_WINDOW_FRAMES            TASK2_A_LINE_WINDOW_FRAMES
#define TASK5_A_LINE_REQUIRED_FRAMES          TASK2_A_LINE_REQUIRED_FRAMES

#define TASK5_POST_LINE_DISTANCE_MM           ((uint32_t)300U)
#define TASK5_BALL_TARGET_X                   BALL_BALANCE_TARGET_CX_B

#define TASK5_STOP_DECEL_MM_S2                ((int32_t)300)
#define TASK5_STOP_JERK_MM_S3                 ((int32_t)1500)

/* The scored lap time ends at A; post-line travel has its own timeout. */
#define TASK5_LAP_TIMEOUT_MS                  ((uint32_t)30000U)
#define TASK5_POST_LINE_TIMEOUT_MS            ((uint32_t)8000U)
#define TASK5_STOP_TIMEOUT_MS                 ((uint32_t)5000U)
#define TASK5_REPORT_PERIOD_MS                ((uint32_t)200U)

#endif /* TASK5_LAP_HOLD_CONFIG_H */
