#ifndef BALL_BALANCE_CONTROL_CONFIG_H
#define BALL_BALANCE_CONTROL_CONFIG_H

#include <stdint.h>

/* 2ebc854 cyclic test targets in the 1280x960 vision coordinate system. */
#define BALL_BALANCE_TARGET_CX_A                    ((int32_t)1000)
#define BALL_BALANCE_TARGET_CX_B                    ((int32_t)600)
#define BALL_BALANCE_TARGET_HOLD_MS                 ((uint32_t)5000U)
#define BALL_BALANCE_VISION_X_MAX                   ((int32_t)1279)

/*
 * 7a35ce0 3-phase cycle targets: 0cm -> +5cm -> -5cm -> repeat
 * Phase 0: 0cm (653),  hold 5s
 * Phase 1: +5cm (869), bounce (no hold)
 * Phase 2: -5cm (447), hold 5s
 */
#define BALL_BALANCE_TARGET_CX_0                    ((int32_t)653)
#define BALL_BALANCE_TARGET_CX_P5                   ((int32_t)869)
#define BALL_BALANCE_TARGET_CX_N5                   ((int32_t)447)
#define BALL_BALANCE_CYCLE_HOLD_MS                  ((uint32_t)5000U)

/*
 * Servo calibration from 7a35ce0.
 * Center = 1650 us, push range = 1550-1750 us.
 */
#define BALL_BALANCE_SERVO_CENTER_US                ((uint16_t)1650U)
#define BALL_BALANCE_SERVO_PUSH_MIN_US              ((uint16_t)1550U)
#define BALL_BALANCE_SERVO_PUSH_MAX_US              ((uint16_t)1750U)
#define BALL_BALANCE_SERVO_BRAKE_MIN_US             ((uint16_t)1300U)
#define BALL_BALANCE_SERVO_BRAKE_MAX_US             ((uint16_t)2000U)

/* Push gain 2.0, represented as Q10 to avoid floating-point work. */
#define BALL_BALANCE_PUSH_KP_Q10                    ((int32_t)2048)
#define BALL_BALANCE_GAIN_Q_SHIFT                   10

/*
 * Brake distance gain.
 * Default = 5 (7a35ce0: phases 0/1).
 * Phase 2 (-5cm) uses doubled gain = 10 for earlier braking.
 * Runtime-settable via BallBalanceControl_SetBrakeGain().
 */
#define BALL_BALANCE_BRAKE_DISTANCE_GAIN            ((int32_t)5)
#define BALL_BALANCE_BRAKE_DISTANCE_GAIN_N5         ((int32_t)10)

/* 7a35ce0 braking law: min(60 + |speed| * 4, 300) us. */
#define BALL_BALANCE_BRAKE_BASE_US                  ((int32_t)60)
#define BALL_BALANCE_BRAKE_SPEED_NUMERATOR          ((int32_t)4)
#define BALL_BALANCE_BRAKE_SPEED_DENOMINATOR        ((int32_t)1)
#define BALL_BALANCE_BRAKE_MAX_US                   ((int32_t)300)

/*
 * 7a35ce0 burst-based stuck detection:
 * Wait STUCK_WAIT frames of low speed, then apply STUCK_BOOST_US
 * for STUCK_HOLD frames. Reset counter when speed recovers or braking.
 */
#define BALL_BALANCE_STUCK_SPEED_PX                 ((int32_t)3)
#define BALL_BALANCE_STUCK_WAIT                     ((uint32_t)15U)
#define BALL_BALANCE_STUCK_BOOST_US                 ((uint32_t)100U)
#define BALL_BALANCE_STUCK_HOLD                     ((uint32_t)8U)

#define BALL_BALANCE_DEADZONE_PX                    ((int32_t)40)
#define BALL_BALANCE_MIN_STEP_US                    ((int32_t)20)
#define BALL_BALANCE_LOST_TIMEOUT_FRAMES            ((uint32_t)10U)

#endif /* BALL_BALANCE_CONTROL_CONFIG_H */
