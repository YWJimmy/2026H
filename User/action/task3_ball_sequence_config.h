#ifndef TASK3_BALL_SEQUENCE_CONFIG_H
#define TASK3_BALL_SEQUENCE_CONFIG_H

#include <stdint.h>

#define TASK3_TARGET_O_X                  ((int32_t)653)
#define TASK3_TARGET_POS5_X               ((int32_t)869)
#define TASK3_TARGET_NEG5_X               ((int32_t)447)

#define TASK3_O_TOLERANCE_MM              ((int32_t)7)
#define TASK3_POS5_TOLERANCE_MM           ((int32_t)7)
#define TASK3_NEG5_TOLERANCE_MM           ((int32_t)10)
#define TASK3_SETTLE_SPEED_MM_S            ((int32_t)15)

#define TASK3_O_STABLE_FRAMES             ((uint8_t)3U)
#define TASK3_POS5_STABLE_FRAMES          ((uint8_t)1U)
#define TASK3_NEG5_STABLE_FRAMES          ((uint8_t)3U)

#define TASK3_VISION_TIMEOUT_MS            ((uint32_t)500U)
#define TASK3_O_TIMEOUT_MS                 ((uint32_t)1500U)
#define TASK3_POS5_TIMEOUT_MS              ((uint32_t)2500U)
#define TASK3_NEG5_TIMEOUT_MS              ((uint32_t)3500U)
#define TASK3_RUN_LIMIT_MS                 ((uint32_t)5000U)
#define TASK3_REPORT_PERIOD_MS             ((uint32_t)100U)

#endif /* TASK3_BALL_SEQUENCE_CONFIG_H */
