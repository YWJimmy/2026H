#ifndef BALL_DYNAMICS_MODEL_CONFIG_H
#define BALL_DYNAMICS_MODEL_CONFIG_H

#include <stdint.h>

#define BALL_DYNAMICS_GRAVITY_MM_S2             ((int32_t)9810)
#define BALL_DYNAMICS_KAPPA_Q10                 ((int32_t)731) /* 5/7 */
#define BALL_DYNAMICS_Q_SHIFT                   10

/* Standard 500..2500 us / 180 degree servo, with 1700 us as level. */
#define BALL_DYNAMICS_LEVEL_PULSE_US            ((int32_t)1700)
#define BALL_DYNAMICS_POS_PULSE_PER_MRAD_Q10    ((int32_t)652) /* 0.6366 us/mrad */
#define BALL_DYNAMICS_NEG_PULSE_PER_MRAD_Q10    ((int32_t)652)
#define BALL_DYNAMICS_SERVO_DIRECTION           ((int32_t)-1)
#define BALL_DYNAMICS_MIN_PULSE_US              ((int32_t)1300)
#define BALL_DYNAMICS_MAX_PULSE_US              ((int32_t)2000)
#define BALL_DYNAMICS_MAX_ANGLE_MRAD             ((int32_t)628)

/* Rail axis defaults to chassis longitudinal direction. */
#define BALL_DYNAMICS_AXIS_FORWARD_Q10           ((int32_t)1024)
#define BALL_DYNAMICS_AXIS_LATERAL_Q10           ((int32_t)0)
#define BALL_DYNAMICS_MEASURED_ACCEL_BLEND_Q10   ((int32_t)256)

#endif /* BALL_DYNAMICS_MODEL_CONFIG_H */
