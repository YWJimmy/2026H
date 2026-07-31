#ifndef BALL_BALANCE_CONTROL_CONFIG_H
#define BALL_BALANCE_CONTROL_CONFIG_H

#include <stdint.h>

/* 2ebc854 cyclic test targets in the 1280x960 vision coordinate system. */
#define BALL_BALANCE_TARGET_CX_A                    ((int32_t)1000)
#define BALL_BALANCE_TARGET_CX_B                    ((int32_t)600)
#define BALL_BALANCE_TARGET_HOLD_MS                 ((uint32_t)5000U)
#define BALL_BALANCE_VISION_X_MAX                   ((int32_t)1279)

/* Servo calibration and output limits from 2ebc854. */
#define BALL_BALANCE_SERVO_CENTER_US                ((uint16_t)1700U)
#define BALL_BALANCE_SERVO_PUSH_MIN_US              ((uint16_t)1300U)
#define BALL_BALANCE_SERVO_PUSH_MAX_US              ((uint16_t)2000U)
#define BALL_BALANCE_SERVO_BRAKE_MIN_US             ((uint16_t)1300U)
#define BALL_BALANCE_SERVO_BRAKE_MAX_US             ((uint16_t)2000U)

/* Push gain 1.0, represented as Q10 to avoid floating-point work. */
#define BALL_BALANCE_PUSH_KP_Q10                    ((int32_t)1024)
#define BALL_BALANCE_GAIN_Q_SHIFT                   10

/* Start reverse braking when |error| < |speed| * 5. */
#define BALL_BALANCE_BRAKE_DISTANCE_GAIN            ((int32_t)5)

/* 2ebc854 braking law: min(200 + |speed| * 3 / 2, 120) us. */
#define BALL_BALANCE_BRAKE_BASE_US                  ((int32_t)200)
#define BALL_BALANCE_BRAKE_SPEED_NUMERATOR          ((int32_t)3)
#define BALL_BALANCE_BRAKE_SPEED_DENOMINATOR        ((int32_t)2)
#define BALL_BALANCE_BRAKE_MAX_US                   ((int32_t)120)

/* Low-speed stuck detection and per-frame push boost. */
#define BALL_BALANCE_STUCK_SPEED_PX                 ((int32_t)3)
#define BALL_BALANCE_STUCK_BOOST_US                 ((uint32_t)5U)

#define BALL_BALANCE_DEADZONE_PX                    ((int32_t)30)
#define BALL_BALANCE_MIN_STEP_US                    ((int32_t)20)
#define BALL_BALANCE_LOST_TIMEOUT_FRAMES            ((uint32_t)10U)

#endif /* BALL_BALANCE_CONTROL_CONFIG_H */
