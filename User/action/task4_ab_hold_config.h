#ifndef TASK4_AB_HOLD_CONFIG_H
#define TASK4_AB_HOLD_CONFIG_H

#include <stdint.h>

/* Preserve the frame branch route timing/distance behavior. */
#define TASK4_B_TIME_DISTANCE_MM              ((uint32_t)1500U)
#define TASK4_STOP_START_DISTANCE_MM          ((uint32_t)1800U)
#define TASK4_AB_TIME_LIMIT_MS                ((uint32_t)8000U)

/*
 * B timing lock: distance is authoritative; a confirmed turn is the backup.
 * The distance gate prevents small straight-line corrections from ending the
 * timer early.
 */
#define TASK4_B_TURN_MIN_DISTANCE_MM           ((uint32_t)1100U)
#define TASK4_B_TURN_THRESHOLD_MM_S            ((int32_t)80)
#define TASK4_B_TURN_CONFIRM_MS                ((uint32_t)60U)

/* Start a smooth stop after 1800 mm; keep ball control active until stopped. */
#define TASK4_STOP_DECEL_MM_S2                ((int32_t)300)
#define TASK4_STOP_JERK_MM_S3                 ((int32_t)1500)
#define TASK4_RUN_TIMEOUT_MS                  ((uint32_t)20000U)
#define TASK4_REPORT_PERIOD_MS                ((uint32_t)200U)

#endif /* TASK4_AB_HOLD_CONFIG_H */
