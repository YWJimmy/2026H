#include "ball_position_action.h"

#include "ball_motion_estimator.h"
#include "ball_position_action_config.h"
#include "ball_dynamics_model.h"
#include "vision.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static BallPositionActionCommand_t s_command;
static BallPositionActionStatus_t s_status;
static VisionStatus_t s_vision;
static BallMotionState_t s_motion;
static BallBaseMotion_t s_base_motion;
static BallPositionControllerStatus_t s_control;
static uint32_t s_last_vision_sequence = 0U;
static int64_t s_capture_sum_um = 0;
static int32_t s_capture_min_um = 0;
static int32_t s_capture_max_um = 0;
static int32_t s_target_position_um = 0;

static int32_t BallPosition_AbsI32(int32_t value)
{
    if (value >= 0)
    {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

static BallPositionActionResult_t BallPosition_Fault(uint32_t detail)
{
    s_status.fault_detail = detail;
    s_status.state = BALL_POSITION_ACTION_STATE_FAULT;
    s_status.active = false;
    BallPositionController_Stop();
    Vision_Stop();
    return BALL_POSITION_ACTION_RESULT_FAULT;
}

static bool BallPosition_CommandValid(
    const BallPositionActionCommand_t *command)
{
    if (command == NULL)
    {
        return false;
    }
    if ((!command->capture_current_target) &&
        ((command->target_x < 0) ||
         (command->target_x >=
          (int32_t)VISION_PROTOCOL_FRAME_WIDTH)))
    {
        return false;
    }
    return (command->tolerance_mm >= 0) &&
           (command->settle_speed_mm_s >= 0) &&
           (command->stable_frames > 0U) &&
           (command->capture_frames > 0U) &&
           (command->capture_min_score_milli <= 1000U) &&
           (command->capture_max_spread_mm >= 0);
}

static void BallPosition_ResetTargetState(uint32_t now_ms)
{
    s_status.target_timestamp_ms = now_ms;
    s_status.stable_frame_count = 0U;
    s_status.capture_frame_count = 0U;
    s_status.reached = false;
    s_capture_sum_um = 0;
    s_capture_min_um = 0;
    s_capture_max_um = 0;
}

bool BallPositionAction_Init(void)
{
    memset(&s_command, 0, sizeof(s_command));
    memset(&s_status, 0, sizeof(s_status));
    s_status.initialized = true;
    s_status.state = BALL_POSITION_ACTION_STATE_IDLE;
    s_initialized = true;
    return true;
}

void BallPositionAction_DefaultCommand(
    BallPositionActionCommand_t *command,
    int32_t target_x)
{
    if (command == NULL)
    {
        return;
    }
    command->target_x = target_x;
    command->tolerance_mm = BALL_POSITION_DEFAULT_TOLERANCE_MM;
    command->settle_speed_mm_s =
        BALL_POSITION_DEFAULT_SETTLE_SPEED_MM_S;
    command->stable_frames = BALL_POSITION_DEFAULT_STABLE_FRAMES;
    command->capture_frames = BALL_POSITION_CAPTURE_FRAMES;
    command->capture_min_score_milli =
        BALL_POSITION_CAPTURE_MIN_SCORE_MILLI;
    command->capture_max_spread_mm =
        BALL_POSITION_CAPTURE_MAX_SPREAD_MM;
    command->vision_timeout_ms =
        BALL_POSITION_DEFAULT_VISION_TIMEOUT_MS;
    command->move_timeout_ms = 0U;
    command->capture_current_target = false;
}

static bool BallPosition_ApplyTarget(int32_t target_um)
{
    s_target_position_um = target_um;
    s_status.target_mm = target_um / 1000;
    s_status.target_x =
        BallMotionEstimator_PositionUmToPixel(target_um);
    return BallPositionController_SetTarget(
        target_um,
        s_command.tolerance_mm * 1000,
        s_command.settle_speed_mm_s * 1000);
}

bool BallPositionAction_Start(
    const BallPositionActionCommand_t *command,
    uint32_t now_ms)
{
    int32_t target_um;

    if ((!s_initialized) || !BallPosition_CommandValid(command) ||
        s_status.active)
    {
        s_status.fault_detail = 1U;
        return false;
    }
    if (!Vision_Init())
    {
        s_status.fault_detail = 2U;
        return false;
    }
    if (!BallMotionEstimator_Init(now_ms) ||
        !BallPositionController_Init(now_ms))
    {
        Vision_Stop();
        s_status.fault_detail = 3U;
        return false;
    }

    s_command = *command;
    if (s_command.vision_timeout_ms == 0U)
    {
        s_command.vision_timeout_ms =
            BALL_POSITION_DEFAULT_VISION_TIMEOUT_MS;
    }
    memset(&s_vision, 0, sizeof(s_vision));
    memset(&s_motion, 0, sizeof(s_motion));
    memset(&s_base_motion, 0, sizeof(s_base_motion));
    memset(&s_control, 0, sizeof(s_control));
    s_last_vision_sequence = 0U;
    BallDynamicsModel_ResetBaseMotion(now_ms);

    memset(&s_status, 0, sizeof(s_status));
    s_status.initialized = true;
    s_status.active = true;
    s_status.target_locked = !s_command.capture_current_target;
    s_status.state = s_status.target_locked ?
        BALL_POSITION_ACTION_STATE_MOVING :
        BALL_POSITION_ACTION_STATE_ACQUIRING;
    s_status.start_timestamp_ms = now_ms;
    s_status.target_timestamp_ms = now_ms;
    s_status.last_found_timestamp_ms = now_ms;
    s_status.servo_pulse_us = 1700U;
    BallPosition_ResetTargetState(now_ms);

    if (s_status.target_locked)
    {
        target_um = BallMotionEstimator_PixelToPositionUm(
            s_command.target_x);
        if (!BallPosition_ApplyTarget(target_um))
        {
            (void)BallPosition_Fault(4U);
            return false;
        }
    }
    return true;
}

bool BallPositionAction_Retarget(
    const BallPositionActionCommand_t *command,
    uint32_t now_ms)
{
    if ((!s_initialized) || !s_status.active ||
        !BallPosition_CommandValid(command))
    {
        return false;
    }
    s_command = *command;
    if (s_command.vision_timeout_ms == 0U)
    {
        s_command.vision_timeout_ms =
            BALL_POSITION_DEFAULT_VISION_TIMEOUT_MS;
    }
    s_status.target_locked = !s_command.capture_current_target;
    s_status.state = s_status.target_locked ?
        BALL_POSITION_ACTION_STATE_MOVING :
        BALL_POSITION_ACTION_STATE_ACQUIRING;
    BallPosition_ResetTargetState(now_ms);
    if (s_status.target_locked &&
        !BallPosition_ApplyTarget(
            BallMotionEstimator_PixelToPositionUm(
                s_command.target_x)))
    {
        (void)BallPosition_Fault(5U);
        return false;
    }
    return true;
}

static bool BallPosition_UpdateCapture(void)
{
    int32_t spread_um;
    int32_t target_um;

    if (!s_motion.measurement_accepted ||
        (s_motion.confidence_milli <
         s_command.capture_min_score_milli) ||
        (BallPosition_AbsI32(s_motion.velocity_um_s) >
         BALL_POSITION_CAPTURE_MAX_SPEED_MM_S * 1000))
    {
        s_status.capture_frame_count = 0U;
        s_capture_sum_um = 0;
        return true;
    }

    if (s_status.capture_frame_count == 0U)
    {
        s_capture_min_um = s_motion.position_um;
        s_capture_max_um = s_motion.position_um;
    }
    else
    {
        if (s_motion.position_um < s_capture_min_um)
        {
            s_capture_min_um = s_motion.position_um;
        }
        if (s_motion.position_um > s_capture_max_um)
        {
            s_capture_max_um = s_motion.position_um;
        }
    }
    spread_um = s_capture_max_um - s_capture_min_um;
    if (spread_um > s_command.capture_max_spread_mm * 1000)
    {
        s_status.capture_frame_count = 1U;
        s_capture_sum_um = s_motion.position_um;
        s_capture_min_um = s_motion.position_um;
        s_capture_max_um = s_motion.position_um;
        return true;
    }

    s_capture_sum_um += s_motion.position_um;
    s_status.capture_frame_count++;
    if (s_status.capture_frame_count < s_command.capture_frames)
    {
        return true;
    }

    target_um = (int32_t)(s_capture_sum_um /
        s_status.capture_frame_count);
    if (!BallPosition_ApplyTarget(target_um))
    {
        return false;
    }
    s_status.target_locked = true;
    s_status.state = BALL_POSITION_ACTION_STATE_MOVING;
    s_status.target_timestamp_ms = s_motion.timestamp_ms;
    s_command.capture_current_target = false;
    return true;
}

static void BallPosition_CopyStatus(void)
{
    s_status.estimator_valid = s_motion.valid;
    s_status.center_x = s_motion.center_x_px;
    s_status.position_mm = s_motion.position_um / 1000;
    s_status.error_mm =
        (s_motion.position_um - s_target_position_um) / 1000;
    s_status.velocity_mm_s = s_motion.velocity_um_s / 1000;
    s_status.estimated_accel_mm_s2 = s_motion.model_accel_um_s2 / 1000;
    s_status.chassis_ff_accel_mm_s2 =
        s_control.chassis_axis_accel_mm_s2;
    s_status.platform_angle_mrad =
        s_control.desired_platform_angle_mrad;
    s_status.desired_ball_accel_mm_s2 =
        s_control.desired_ball_accel_mm_s2;
    s_status.servo_pulse_us = s_control.servo_pulse_us;
    s_status.control_mode = s_control.mode;
    s_status.confidence_milli = s_motion.confidence_milli;
    s_status.accepted_frames = s_motion.accepted_frames;
    s_status.rejected_frames = s_motion.rejected_frames;
}

BallPositionActionResult_t BallPositionAction_Update(uint32_t now_ms)
{
    bool new_frame;

    if ((!s_initialized) ||
        (s_status.state == BALL_POSITION_ACTION_STATE_FAULT))
    {
        return BALL_POSITION_ACTION_RESULT_FAULT;
    }
    if (!s_status.active)
    {
        return BallPosition_Fault(6U);
    }

    Vision_Update();
    if (!Vision_GetStatus(&s_vision) ||
        !BallDynamicsModel_GetBaseMotion(now_ms, &s_base_motion))
    {
        return BallPosition_Fault(7U);
    }
    new_frame = s_vision.has_frame &&
        (s_vision.sequence != s_last_vision_sequence);
    if (new_frame)
    {
        s_last_vision_sequence = s_vision.sequence;
    }
    if (!BallMotionEstimator_Update(
            now_ms,
            BallPositionController_GetPredictedAccelMmps2(),
            new_frame,
            s_vision.sequence,
            s_vision.timestamp_ms,
            new_frame ? &s_vision.frame : NULL) ||
        !BallMotionEstimator_GetState(&s_motion))
    {
        return BallPosition_Fault(8U);
    }

    s_status.vision_data_valid = s_vision.data_valid;
    s_status.vision_found = s_vision.has_frame &&
        s_vision.frame.found;
    s_status.vision_sequence = s_vision.sequence;
    if (s_motion.measurement_accepted)
    {
        s_status.last_found_timestamp_ms = now_ms;
    }

    if (!s_status.target_locked && new_frame &&
        !BallPosition_UpdateCapture())
    {
        return BallPosition_Fault(9U);
    }
    if (!BallPositionController_Update(
            now_ms,
            &s_motion,
            &s_base_motion) ||
        !BallPositionController_GetStatus(&s_control))
    {
        return BallPosition_Fault(10U);
    }
    BallPosition_CopyStatus();

    if (new_frame && s_motion.measurement_accepted &&
        s_status.target_locked)
    {
        if ((BallPosition_AbsI32(
                 s_motion.position_um - s_target_position_um) <=
             s_command.tolerance_mm * 1000) &&
            (BallPosition_AbsI32(s_motion.velocity_um_s) <=
             s_command.settle_speed_mm_s * 1000))
        {
            s_status.stable_frame_count++;
        }
        else
        {
            s_status.stable_frame_count = 0U;
            s_status.reached = false;
            s_status.state = BALL_POSITION_ACTION_STATE_MOVING;
        }
        if (s_status.stable_frame_count >= s_command.stable_frames)
        {
            s_status.reached = true;
            s_status.state = BALL_POSITION_ACTION_STATE_HOLDING;
        }
    }

    if ((uint32_t)(now_ms - s_status.last_found_timestamp_ms) >=
        s_command.vision_timeout_ms)
    {
        return BallPosition_Fault(11U);
    }
    if (!s_status.reached && (s_command.move_timeout_ms > 0U) &&
        ((uint32_t)(now_ms - s_status.target_timestamp_ms) >=
         s_command.move_timeout_ms))
    {
        return BallPosition_Fault(12U);
    }
    return s_status.reached ?
        BALL_POSITION_ACTION_RESULT_REACHED :
        BALL_POSITION_ACTION_RESULT_RUNNING;
}

void BallPositionAction_Stop(void)
{
    BallPositionController_Stop();
    Vision_Stop();
    s_status.active = false;
    s_status.target_locked = false;
    s_status.reached = false;
    s_status.state = BALL_POSITION_ACTION_STATE_IDLE;
}

void BallPositionAction_ForceSafeStop(void)
{
    BallPositionAction_Stop();
}

bool BallPositionAction_IsActive(void)
{
    return s_initialized && s_status.active;
}

bool BallPositionAction_GetStatus(BallPositionActionStatus_t *status)
{
    if ((!s_initialized) || (status == NULL))
    {
        return false;
    }
    *status = s_status;
    return true;
}

uint32_t BallPositionAction_GetFaultDetail(void)
{
    return s_status.fault_detail;
}

const char *BallPositionAction_StateName(BallPositionActionState_t state)
{
    switch (state)
    {
        case BALL_POSITION_ACTION_STATE_IDLE:
            return "IDLE";
        case BALL_POSITION_ACTION_STATE_ACQUIRING:
            return "ACQUIRE";
        case BALL_POSITION_ACTION_STATE_MOVING:
            return "MOVING";
        case BALL_POSITION_ACTION_STATE_HOLDING:
            return "HOLDING";
        case BALL_POSITION_ACTION_STATE_FAULT:
        default:
            return "FAULT";
    }
}
