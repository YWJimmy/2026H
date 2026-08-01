#include "ball_dynamics_model.h"

#include "ball_dynamics_model_config.h"
#include "chassis.h"
#include "chassis_config.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_have_chassis_sample = false;
static uint32_t s_previous_control_sequence = 0U;
static uint32_t s_previous_timestamp_ms = 0U;
static int32_t s_previous_forward_mm_s = 0;
static int32_t s_previous_yaw_mrad_s = 0;
static BallBaseMotion_t s_last_motion;

static int32_t BallDynamics_ClampI64(int64_t value)
{
    if (value > INT32_MAX)
    {
        return INT32_MAX;
    }
    if (value < INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static int32_t BallDynamics_ClampI32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void BallDynamicsModel_ResetBaseMotion(uint32_t now_ms)
{
    s_have_chassis_sample = false;
    s_previous_control_sequence = 0U;
    s_previous_timestamp_ms = now_ms;
    s_previous_forward_mm_s = 0;
    s_previous_yaw_mrad_s = 0;
    memset(&s_last_motion, 0, sizeof(s_last_motion));
    s_last_motion.timestamp_ms = now_ms;
}

bool BallDynamicsModel_GetBaseMotion(
    uint32_t now_ms,
    BallBaseMotion_t *motion)
{
    ChassisStatus_t chassis;
    uint32_t dt_ms;
    int32_t forward_measured;
    int32_t yaw_mrad_s;
    int32_t measured_accel = 0;
    int32_t yaw_accel = 0;
    int32_t planned_accel;
    int32_t blended_accel;

    if (motion == NULL)
    {
        return false;
    }
    memset(motion, 0, sizeof(*motion));
    motion->timestamp_ms = now_ms;

    if (!Chassis_IsInitialized() ||
        !Chassis_GetStatus(&chassis))
    {
        return true;
    }

    if (s_have_chassis_sample &&
        (chassis.control_sequence ==
         s_previous_control_sequence))
    {
        *motion = s_last_motion;
        motion->timestamp_ms = now_ms;
        return true;
    }

    forward_measured =
        (chassis.left_measured_mm_s +
         chassis.right_measured_mm_s) / 2;
    yaw_mrad_s = BallDynamics_ClampI64(
        ((int64_t)(chassis.right_measured_mm_s -
                   chassis.left_measured_mm_s) * 1000LL) /
        CHASSIS_TRACK_WIDTH_MM);
    dt_ms = chassis.timestamp_ms - s_previous_timestamp_ms;
    if (s_have_chassis_sample && (dt_ms > 0U) &&
        (dt_ms <= 100U))
    {
        measured_accel = BallDynamics_ClampI64(
            ((int64_t)(forward_measured -
                       s_previous_forward_mm_s) * 1000LL) /
            dt_ms);
        yaw_accel = BallDynamics_ClampI64(
            ((int64_t)(yaw_mrad_s -
                       s_previous_yaw_mrad_s) * 1000LL) /
            dt_ms);
    }

    planned_accel = chassis.forward_accel_mm_s2;
    blended_accel = planned_accel +
        (int32_t)(((int64_t)BALL_DYNAMICS_MEASURED_ACCEL_BLEND_Q10 *
                   (measured_accel - planned_accel)) >>
                  BALL_DYNAMICS_Q_SHIFT);

    motion->valid = chassis.enabled &&
        chassis.encoder_sample_valid;
    motion->timestamp_ms = chassis.timestamp_ms;
    motion->forward_velocity_mm_s = forward_measured;
    motion->forward_accel_mm_s2 = blended_accel;
    motion->yaw_rate_mrad_s = yaw_mrad_s;
    motion->yaw_accel_mrad_s2 = yaw_accel;
    motion->lateral_accel_mm_s2 = BallDynamics_ClampI64(
        ((int64_t)forward_measured * yaw_mrad_s) / 1000LL);
    motion->axis_accel_mm_s2 = BallDynamics_ClampI64(
        (((int64_t)BALL_DYNAMICS_AXIS_FORWARD_Q10 * blended_accel) +
         ((int64_t)BALL_DYNAMICS_AXIS_LATERAL_Q10 *
          motion->lateral_accel_mm_s2)) >>
        BALL_DYNAMICS_Q_SHIFT);

    s_have_chassis_sample = true;
    s_previous_control_sequence = chassis.control_sequence;
    s_previous_timestamp_ms = chassis.timestamp_ms;
    s_previous_forward_mm_s = forward_measured;
    s_previous_yaw_mrad_s = yaw_mrad_s;
    s_last_motion = *motion;
    return true;
}

int32_t BallDynamicsModel_DesiredAccelToAngleMrad(
    int32_t desired_ball_accel_mm_s2,
    int32_t chassis_axis_accel_mm_s2)
{
    int64_t desired_over_kappa =
        ((int64_t)desired_ball_accel_mm_s2 <<
         BALL_DYNAMICS_Q_SHIFT) /
        BALL_DYNAMICS_KAPPA_Q10;
    int32_t angle = BallDynamics_ClampI64(
        ((chassis_axis_accel_mm_s2 + desired_over_kappa) *
         1000LL) / BALL_DYNAMICS_GRAVITY_MM_S2);

    return BallDynamics_ClampI32(
        angle,
        -BALL_DYNAMICS_MAX_ANGLE_MRAD,
        BALL_DYNAMICS_MAX_ANGLE_MRAD);
}

int32_t BallDynamicsModel_PredictBallAccelMmps2(
    int32_t platform_angle_mrad,
    int32_t chassis_axis_accel_mm_s2)
{
    int64_t gravity_component =
        ((int64_t)BALL_DYNAMICS_GRAVITY_MM_S2 *
         platform_angle_mrad) / 1000LL;

    return BallDynamics_ClampI64(
        ((gravity_component - chassis_axis_accel_mm_s2) *
         BALL_DYNAMICS_KAPPA_Q10) >>
        BALL_DYNAMICS_Q_SHIFT);
}

uint16_t BallDynamicsModel_AngleToPulseUs(int32_t angle_mrad)
{
    int32_t gain_q10 = (angle_mrad >= 0) ?
        BALL_DYNAMICS_POS_PULSE_PER_MRAD_Q10 :
        BALL_DYNAMICS_NEG_PULSE_PER_MRAD_Q10;
    int32_t pulse = BALL_DYNAMICS_LEVEL_PULSE_US +
        BALL_DYNAMICS_SERVO_DIRECTION *
        (int32_t)(((int64_t)angle_mrad *
                   gain_q10) >>
                  BALL_DYNAMICS_Q_SHIFT);

    pulse = BallDynamics_ClampI32(
        pulse,
        BALL_DYNAMICS_MIN_PULSE_US,
        BALL_DYNAMICS_MAX_PULSE_US);
    return (uint16_t)pulse;
}

int32_t BallDynamicsModel_PulseToAngleMrad(uint16_t pulse_us)
{
    int32_t delta = (int32_t)pulse_us -
        BALL_DYNAMICS_LEVEL_PULSE_US;
    int32_t angle_sign = delta * BALL_DYNAMICS_SERVO_DIRECTION;
    int32_t gain_q10 = (angle_sign >= 0) ?
        BALL_DYNAMICS_POS_PULSE_PER_MRAD_Q10 :
        BALL_DYNAMICS_NEG_PULSE_PER_MRAD_Q10;

    return BallDynamics_ClampI64(
        (((int64_t)delta << BALL_DYNAMICS_Q_SHIFT) /
         gain_q10) *
        BALL_DYNAMICS_SERVO_DIRECTION);
}
