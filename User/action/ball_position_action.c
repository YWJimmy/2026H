#include "ball_position_action.h"

#include "ball_position_action_config.h"
#include "ball_balance_control_config.h"
#include "vision.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static BallPositionActionCommand_t s_command;
static BallPositionActionStatus_t s_status;
static VisionStatus_t s_vision;
static BallBalanceControlStatus_t s_control;
static uint32_t s_last_vision_sequence = 0U;
static int32_t s_previous_center_x = 0;

static int32_t BallPosition_AbsI32(int32_t value)
{
    if (value >= 0)
    {
        return value;
    }
    if (value == INT32_MIN)
    {
        return INT32_MAX;
    }
    return -value;
}

static BallPositionActionResult_t BallPosition_Fault(
    uint32_t detail)
{
    s_status.fault_detail = detail;
    s_status.state = BALL_POSITION_ACTION_STATE_FAULT;
    s_status.active = false;
    BallBalanceControl_Stop();
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
         (command->target_x > BALL_BALANCE_VISION_X_MAX)))
    {
        return false;
    }
    return (command->tolerance_px >= 0) &&
           (command->settle_speed_px >= 0) &&
           (command->stable_frames > 0U);
}

static void BallPosition_ResetTargetState(uint32_t now_ms)
{
    s_status.target_timestamp_ms = now_ms;
    s_status.stable_frame_count = 0U;
    s_status.reached = false;
    s_status.raw_speed_px = 0;
    s_status.filtered_speed_px = 0;
    s_previous_center_x = 0;
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
    command->tolerance_px =
        BALL_POSITION_DEFAULT_TOLERANCE_PX;
    command->settle_speed_px =
        BALL_POSITION_DEFAULT_SETTLE_SPEED_PX;
    command->stable_frames =
        BALL_POSITION_DEFAULT_STABLE_FRAMES;
    command->vision_timeout_ms =
        BALL_POSITION_DEFAULT_VISION_TIMEOUT_MS;
    command->move_timeout_ms = 0U;
    command->capture_current_target = false;
}

bool BallPositionAction_Start(
    const BallPositionActionCommand_t *command,
    uint32_t now_ms)
{
    if ((!s_initialized) ||
        !BallPosition_CommandValid(command) ||
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
    if (!BallBalanceControl_Init())
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
    memset(&s_control, 0, sizeof(s_control));
    s_last_vision_sequence = 0U;
    s_previous_center_x = 0;

    s_status.active = true;
    s_status.target_locked =
        !s_command.capture_current_target;
    s_status.reached = false;
    s_status.vision_data_valid = false;
    s_status.vision_found = false;
    s_status.state = s_status.target_locked ?
        BALL_POSITION_ACTION_STATE_MOVING :
        BALL_POSITION_ACTION_STATE_ACQUIRING;
    s_status.start_timestamp_ms = now_ms;
    s_status.target_timestamp_ms = now_ms;
    s_status.last_found_timestamp_ms = now_ms;
    s_status.vision_sequence = 0U;
    s_status.stable_frame_count = 0U;
    s_status.fault_detail = 0U;
    s_status.center_x = 0;
    s_status.target_x = s_status.target_locked ?
        s_command.target_x : 0;
    s_status.error_px = 0;
    s_status.raw_speed_px = 0;
    s_status.filtered_speed_px = 0;
    s_status.servo_pulse_us =
        BALL_BALANCE_SERVO_CENTER_US;
    s_status.control_mode =
        BALL_BALANCE_MODE_WAITING_VISION;

    if (s_status.target_locked &&
        !BallBalanceControl_SetTargetX(
            s_command.target_x))
    {
        (void)BallPosition_Fault(4U);
        return false;
    }
    return true;
}

bool BallPositionAction_Retarget(
    const BallPositionActionCommand_t *command,
    uint32_t now_ms)
{
    if ((!s_initialized) ||
        (!s_status.active) ||
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
    s_status.target_locked =
        !s_command.capture_current_target;
    s_status.target_x = s_status.target_locked ?
        s_command.target_x : 0;
    s_status.state = s_status.target_locked ?
        BALL_POSITION_ACTION_STATE_MOVING :
        BALL_POSITION_ACTION_STATE_ACQUIRING;
    BallPosition_ResetTargetState(now_ms);

    if (s_status.target_locked &&
        !BallBalanceControl_SetTargetX(
            s_command.target_x))
    {
        (void)BallPosition_Fault(5U);
        return false;
    }
    return true;
}

BallPositionActionResult_t BallPositionAction_Update(
    uint32_t now_ms)
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
    if (!Vision_GetStatus(&s_vision))
    {
        return BallPosition_Fault(7U);
    }

    s_status.vision_data_valid = s_vision.data_valid;
    s_status.vision_found =
        s_vision.has_frame && s_vision.frame.found;
    s_status.vision_sequence = s_vision.sequence;
    new_frame = s_vision.has_frame &&
        (s_vision.sequence != s_last_vision_sequence);

    /* Lock a captured target before the first control calculation. */
    if (new_frame && s_vision.frame.found &&
        !s_status.target_locked)
    {
        int32_t captured_x =
            (int32_t)s_vision.frame.center_x;

        s_command.target_x = captured_x;
        s_command.capture_current_target = false;
        if (!BallBalanceControl_SetTargetX(captured_x))
        {
            return BallPosition_Fault(10U);
        }
        s_status.target_x = captured_x;
        s_status.target_locked = true;
        s_status.state =
            BALL_POSITION_ACTION_STATE_MOVING;
        BallPosition_ResetTargetState(now_ms);
    }

    if (!BallBalanceControl_Update(&s_vision))
    {
        return BallPosition_Fault(8U);
    }
    if (!BallBalanceControl_GetStatus(&s_control))
    {
        return BallPosition_Fault(9U);
    }

    s_status.control_mode = s_control.mode;
    s_status.servo_pulse_us =
        s_control.servo_pulse_us;

    if (new_frame)
    {
        s_last_vision_sequence = s_vision.sequence;
        if (s_vision.frame.found)
        {
            int32_t center_x =
                (int32_t)s_vision.frame.center_x;

            s_status.last_found_timestamp_ms = now_ms;
            s_status.center_x = center_x;
            if (s_previous_center_x != 0)
            {
                s_status.raw_speed_px =
                    center_x - s_previous_center_x;
                s_status.filtered_speed_px =
                    (s_status.filtered_speed_px +
                     s_status.raw_speed_px) / 2;
            }
            else
            {
                s_status.raw_speed_px = 0;
                s_status.filtered_speed_px = 0;
            }
            s_previous_center_x = center_x;

            s_status.error_px =
                s_status.center_x - s_status.target_x;

            if ((BallPosition_AbsI32(s_status.error_px) <=
                 s_command.tolerance_px) &&
                (BallPosition_AbsI32(s_status.raw_speed_px) <=
                 s_command.settle_speed_px) &&
                (BallPosition_AbsI32(
                    s_status.filtered_speed_px) <=
                 s_command.settle_speed_px))
            {
                s_status.stable_frame_count++;
            }
            else
            {
                s_status.stable_frame_count = 0U;
                s_status.reached = false;
                s_status.state =
                    BALL_POSITION_ACTION_STATE_MOVING;
            }

            if (s_status.stable_frame_count >=
                s_command.stable_frames)
            {
                s_status.reached = true;
                s_status.state =
                    BALL_POSITION_ACTION_STATE_HOLDING;
            }
        }
    }

    if ((uint32_t)(now_ms -
            s_status.last_found_timestamp_ms) >=
        s_command.vision_timeout_ms)
    {
        return BallPosition_Fault(11U);
    }

    if ((!s_status.reached) &&
        (s_command.move_timeout_ms > 0U) &&
        ((uint32_t)(now_ms -
            s_status.target_timestamp_ms) >=
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
    BallBalanceControl_Stop();
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

bool BallPositionAction_GetStatus(
    BallPositionActionStatus_t *status)
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

const char *BallPositionAction_StateName(
    BallPositionActionState_t state)
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
