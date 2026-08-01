#ifndef TASK4_AB_HOLD_CONFIG_H
#define TASK4_AB_HOLD_CONFIG_H

#include <stdint.h>

#include "ball_motion_estimator_config.h"

#define TASK4_B_TIME_DISTANCE_MM              ((uint32_t)1500U)
#define TASK4_STOP_START_DISTANCE_MM          ((uint32_t)1800U)

/* O 点沿用现有视觉中心标定。 */
#define TASK4_BALL_TARGET_X                   BALL_CALIBRATION_ORIGIN_PX

/* 1800 mm 后开始柔和减速，钢球控制保持到车辆完全停止。 */
#define TASK4_STOP_DECEL_MM_S2                ((int32_t)300)
#define TASK4_STOP_JERK_MM_S3                 ((int32_t)1500)

#define TASK4_RUN_TIMEOUT_MS                  ((uint32_t)20000U)
#define TASK4_REPORT_PERIOD_MS                ((uint32_t)200U)

#endif /* TASK4_AB_HOLD_CONFIG_H */
