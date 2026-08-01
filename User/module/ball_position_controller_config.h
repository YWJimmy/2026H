#ifndef BALL_POSITION_CONTROLLER_CONFIG_H
#define BALL_POSITION_CONTROLLER_CONFIG_H

#include <stdint.h>

#define BALL_CONTROLLER_PERIOD_MS                   ((uint32_t)10U)
/* Derived to reach the existing +/-100 us push range near a 50 mm error. */
#define BALL_CONTROLLER_KP_Q10                      ((int32_t)20480) /* 20.0 s^-2 */
#define BALL_CONTROLLER_KV_Q10                      ((int32_t)9216)  /* 9.0 s^-1 */
#define BALL_CONTROLLER_Q_SHIFT                     10
#define BALL_CONTROLLER_MAX_DESIRED_ACCEL_MM_S2     ((int32_t)1500)
#define BALL_CONTROLLER_MAX_PULSE_STEP_US            ((int32_t)20)

#define BALL_CONTROLLER_STUCK_SPEED_MM_S             ((int32_t)10)
#define BALL_CONTROLLER_STUCK_START_MS               ((uint32_t)250U)
#define BALL_CONTROLLER_STUCK_RAMP_MS                ((uint32_t)500U)
#define BALL_CONTROLLER_STUCK_POS_MAX_ACCEL_MM_S2     ((int32_t)250)
#define BALL_CONTROLLER_STUCK_NEG_MAX_ACCEL_MM_S2     ((int32_t)250)
#define BALL_CONTROLLER_STUCK_RELEASE_SPEED_MM_S      ((int32_t)25)

#endif /* BALL_POSITION_CONTROLLER_CONFIG_H */
