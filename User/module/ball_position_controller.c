#include "ball_position_controller.h"

#include "ball_dynamics_model_config.h"
#include "ball_position_controller_config.h"
#include "bsp_servo.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static BallPositionControllerStatus_t s_status;
static int32_t s_tolerance_um = 0;
static int32_t s_velocity_tolerance_um_s = 0;
static bool s_stuck_tracking = false;
static uint32_t s_stuck_start_ms = 0U;

static int32_t BallController_AbsI32(int32_t value)
{
    if (value >= 0)
    {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

static int32_t BallController_ClampI64(
    int64_t value,
    int32_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return (int32_t)value;
}

static uint16_t BallController_SlewPulse(
    uint16_t current,
    uint16_t target)
{
    int32_t delta = (int32_t)target - (int32_t)current;

    if (delta > BALL_CONTROLLER_MAX_PULSE_STEP_US)
    {
        delta = BALL_CONTROLLER_MAX_PULSE_STEP_US;
    }
    else if (delta < -BALL_CONTROLLER_MAX_PULSE_STEP_US)
    {
        delta = -BALL_CONTROLLER_MAX_PULSE_STEP_US;
    }
    return (uint16_t)((int32_t)current + delta);
}

bool BallPositionController_Init(uint32_t now_ms)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.timestamp_ms = now_ms;
    s_status.servo_pulse_us =
        (uint16_t)BALL_DYNAMICS_LEVEL_PULSE_US;
    s_status.mode = BALL_POSITION_CONTROLLER_IDLE;

    if (!BSP_Servo_Init() ||
        !BSP_Servo_SetPulseUs(s_status.servo_pulse_us) ||
        !BSP_Servo_Enable())
    {
        return false;
    }

    s_status.initialized = true;
    s_status.enabled = true;
    s_status.mode = BALL_POSITION_CONTROLLER_WAITING_STATE;
    return true;
}

bool BallPositionController_SetTarget(
    int32_t target_position_um,
    int32_t tolerance_um,
    int32_t velocity_tolerance_um_s)
{
    if (!s_status.initialized || (tolerance_um < 0) ||
        (velocity_tolerance_um_s < 0))
    {
        return false;
    }
    s_status.target_position_um = target_position_um;
    s_tolerance_um = tolerance_um;
    s_velocity_tolerance_um_s = velocity_tolerance_um_s;
    s_stuck_tracking = false;
    s_status.stuck_duration_ms = 0U;
    return true;
}

static int32_t BallController_StuckCompensation(
    uint32_t now_ms,
    int32_t error_um,
    int32_t velocity_um_s)
{
    uint32_t active_ms;
    int32_t compensation;
    int32_t maximum_compensation;

    if ((BallController_AbsI32(error_um) <= s_tolerance_um) ||
        (BallController_AbsI32(velocity_um_s) >=
         BALL_CONTROLLER_STUCK_RELEASE_SPEED_MM_S * 1000))
    {
        s_stuck_tracking = false;
        s_status.stuck_duration_ms = 0U;
        return 0;
    }
    if (BallController_AbsI32(velocity_um_s) >
        BALL_CONTROLLER_STUCK_SPEED_MM_S * 1000)
    {
        s_stuck_tracking = false;
        s_status.stuck_duration_ms = 0U;
        return 0;
    }
    if (!s_stuck_tracking)
    {
        s_stuck_tracking = true;
        s_stuck_start_ms = now_ms;
        return 0;
    }

    s_status.stuck_duration_ms = now_ms - s_stuck_start_ms;
    if (s_status.stuck_duration_ms <
        BALL_CONTROLLER_STUCK_START_MS)
    {
        return 0;
    }
    active_ms = s_status.stuck_duration_ms -
        BALL_CONTROLLER_STUCK_START_MS;
    if (active_ms > BALL_CONTROLLER_STUCK_RAMP_MS)
    {
        active_ms = BALL_CONTROLLER_STUCK_RAMP_MS;
    }
    maximum_compensation = (error_um > 0) ?
        BALL_CONTROLLER_STUCK_POS_MAX_ACCEL_MM_S2 :
        BALL_CONTROLLER_STUCK_NEG_MAX_ACCEL_MM_S2;
    compensation = (int32_t)(
        ((uint64_t)maximum_compensation *
         active_ms) / BALL_CONTROLLER_STUCK_RAMP_MS);
    return (error_um > 0) ? -compensation : compensation;
}

bool BallPositionController_Update(
    uint32_t now_ms,
    const BallMotionState_t *motion,
    const BallBaseMotion_t *base_motion)
{
    uint32_t dt_ms;
    int64_t desired_accel;
    int32_t target_angle;
    uint16_t target_pulse;

    if (!s_status.initialized || !s_status.enabled ||
        (motion == NULL) || (base_motion == NULL))
    {
        return false;
    }

    dt_ms = now_ms - s_status.timestamp_ms;
    if (dt_ms < BALL_CONTROLLER_PERIOD_MS)
    {
        return true;
    }
    s_status.timestamp_ms = now_ms;
    s_status.chassis_axis_accel_mm_s2 =
        base_motion->axis_accel_mm_s2;

    if (!motion->valid)
    {
        s_status.mode = BALL_POSITION_CONTROLLER_WAITING_STATE;
        target_angle = BallDynamicsModel_DesiredAccelToAngleMrad(
            0,
            base_motion->axis_accel_mm_s2);
    }
    else
    {
        s_status.position_error_um =
            motion->position_um - s_status.target_position_um;
        s_status.at_rest =
            (BallController_AbsI32(s_status.position_error_um) <=
             s_tolerance_um) &&
            (BallController_AbsI32(motion->velocity_um_s) <=
             s_velocity_tolerance_um_s);

        desired_accel =
            -(((int64_t)BALL_CONTROLLER_KP_Q10 *
               s_status.position_error_um) /
              (1000LL << BALL_CONTROLLER_Q_SHIFT));
        desired_accel -=
            (((int64_t)BALL_CONTROLLER_KV_Q10 *
              motion->velocity_um_s) /
             (1000LL << BALL_CONTROLLER_Q_SHIFT));
        s_status.stuck_compensation_mm_s2 =
            BallController_StuckCompensation(
                now_ms,
                s_status.position_error_um,
                motion->velocity_um_s);
        desired_accel += s_status.stuck_compensation_mm_s2;

        /*
         * Preserve the proven e2d738b cruise authority outside the target
         * window.  The model-based velocity term is allowed to command the
         * opposite sign for braking, but a weak command toward the target
         * must still overcome rail and servo static friction.
         */
        if ((s_status.position_error_um < -s_tolerance_um) &&
            (desired_accel > 0) &&
            (desired_accel <
             BALL_CONTROLLER_MIN_DRIVE_ACCEL_MM_S2))
        {
            desired_accel =
                BALL_CONTROLLER_MIN_DRIVE_ACCEL_MM_S2;
        }
        else if ((s_status.position_error_um > s_tolerance_um) &&
                 (desired_accel < 0) &&
                 (desired_accel >
                  -BALL_CONTROLLER_MIN_DRIVE_ACCEL_MM_S2))
        {
            desired_accel =
                -BALL_CONTROLLER_MIN_DRIVE_ACCEL_MM_S2;
        }
        s_status.desired_ball_accel_mm_s2 =
            BallController_ClampI64(
                desired_accel,
                BALL_CONTROLLER_MAX_DESIRED_ACCEL_MM_S2);
        target_angle = BallDynamicsModel_DesiredAccelToAngleMrad(
            s_status.desired_ball_accel_mm_s2,
            base_motion->axis_accel_mm_s2);
        s_status.mode = s_status.at_rest ?
            BALL_POSITION_CONTROLLER_HOLDING :
            BALL_POSITION_CONTROLLER_MOVING;
    }

    target_pulse = BallDynamicsModel_AngleToPulseUs(target_angle);
    s_status.servo_pulse_us = BallController_SlewPulse(
        s_status.servo_pulse_us,
        target_pulse);
    s_status.desired_platform_angle_mrad =
        BallDynamicsModel_PulseToAngleMrad(
            s_status.servo_pulse_us);
    s_status.predicted_ball_accel_mm_s2 =
        BallDynamicsModel_PredictBallAccelMmps2(
            s_status.desired_platform_angle_mrad,
            base_motion->axis_accel_mm_s2);

    if (!BSP_Servo_SetPulseUs(s_status.servo_pulse_us))
    {
        s_status.servo_error_count++;
        s_status.mode = BALL_POSITION_CONTROLLER_SERVO_FAULT;
        return false;
    }
    s_status.control_sequence++;
    return true;
}

void BallPositionController_Stop(void)
{
    if (s_status.initialized)
    {
        (void)BSP_Servo_SetPulseUs(
            (uint16_t)BALL_DYNAMICS_LEVEL_PULSE_US);
        BSP_Servo_Disable();
    }
    s_status.initialized = false;
    s_status.enabled = false;
    s_status.mode = BALL_POSITION_CONTROLLER_IDLE;
}

bool BallPositionController_GetStatus(
    BallPositionControllerStatus_t *status)
{
    if ((!s_status.initialized) || (status == NULL))
    {
        return false;
    }
    *status = s_status;
    return true;
}

int32_t BallPositionController_GetPredictedAccelMmps2(void)
{
    return s_status.predicted_ball_accel_mm_s2;
}

const char *BallPositionController_ModeName(
    BallPositionControllerMode_t mode)
{
    switch (mode)
    {
        case BALL_POSITION_CONTROLLER_IDLE:
            return "IDLE";
        case BALL_POSITION_CONTROLLER_WAITING_STATE:
            return "WAIT";
        case BALL_POSITION_CONTROLLER_MOVING:
            return "MOVE";
        case BALL_POSITION_CONTROLLER_HOLDING:
            return "HOLD";
        case BALL_POSITION_CONTROLLER_SERVO_FAULT:
        default:
            return "SERVO_ERR";
    }
}
