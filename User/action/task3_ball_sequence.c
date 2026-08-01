#include "task3_ball_sequence.h"

#include "ball_position_action.h"
#include "bsp_debug_uart.h"
#include "task3_ball_sequence_config.h"

#include <limits.h>

typedef enum
{
    TASK3_STATE_IDLE = 0,
    TASK3_STATE_WAIT_O,
    TASK3_STATE_TO_POS5,
    TASK3_STATE_TO_NEG5,
    TASK3_STATE_HOLD_NEG5,
    TASK3_STATE_FINISHED,
    TASK3_STATE_FAULT
} Task3State_t;

static bool s_initialized = false;
static Task3State_t s_state = TASK3_STATE_IDLE;
static uint32_t s_start_ms = 0U;
static uint32_t s_motion_start_ms = 0U;
static uint32_t s_result_elapsed_ms = 0U;
static uint32_t s_last_report_ms = 0U;
static uint32_t s_fault_detail = 0U;
static bool s_motion_started = false;
static bool s_result_locked = false;
static BallPositionActionStatus_t s_ball;

static Task3BallSequenceResult_t Task3_Fault(
    uint32_t detail)
{
    s_fault_detail = detail;
    s_state = TASK3_STATE_FAULT;
    BallPositionAction_ForceSafeStop();
    return TASK3_BALL_SEQUENCE_RESULT_FAULT;
}

static void Task3_MakeCommand(
    BallPositionActionCommand_t *command,
    int32_t target_x,
    int32_t tolerance_mm,
    int32_t settle_speed_mm_s,
    uint8_t stable_frames,
    uint32_t timeout_ms)
{
    BallPositionAction_DefaultCommand(command, target_x);
    command->tolerance_mm = tolerance_mm;
    command->settle_speed_mm_s = settle_speed_mm_s;
    command->stable_frames = stable_frames;
    command->vision_timeout_ms =
        TASK3_VISION_TIMEOUT_MS;
    command->move_timeout_ms = timeout_ms;
}

static bool Task3_Retarget(
    int32_t target_x,
    int32_t tolerance_mm,
    int32_t settle_speed_mm_s,
    uint8_t stable_frames,
    uint32_t timeout_ms,
    uint32_t now_ms)
{
    BallPositionActionCommand_t command;

    Task3_MakeCommand(
        &command,
        target_x,
        tolerance_mm,
        settle_speed_mm_s,
        stable_frames,
        timeout_ms);
    return BallPositionAction_Retarget(
        &command,
        now_ms);
}

static void Task3_Report(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_report_ms) <
        TASK3_REPORT_PERIOD_MS)
    {
        return;
    }
    s_last_report_ms = now_ms;

    (void)BSP_Debug_Printf(
        "T3,ST=%u,MS=%lu,MOTION=%lu,CX=%ld,XMM=%ld,TMM=%ld,ERRMM=%ld,VMM=%ld,A=%ld,DES=%ld,FF=%ld,ANG=%ld,SCORE=%u,OK=%lu,REJ=%lu,STABLE=%lu,MODE=%s,PULSE=%u\r\n",
        (unsigned int)s_state,
        (unsigned long)(now_ms - s_start_ms),
        (unsigned long)(s_motion_started ?
            (s_result_locked ? s_result_elapsed_ms :
             (now_ms - s_motion_start_ms)) : 0U),
        (long)s_ball.center_x,
        (long)s_ball.position_mm,
        (long)s_ball.target_mm,
        (long)s_ball.error_mm,
        (long)s_ball.velocity_mm_s,
        (long)s_ball.estimated_accel_mm_s2,
        (long)s_ball.desired_ball_accel_mm_s2,
        (long)s_ball.chassis_ff_accel_mm_s2,
        (long)s_ball.platform_angle_mrad,
        (unsigned int)s_ball.confidence_milli,
        (unsigned long)s_ball.accepted_frames,
        (unsigned long)s_ball.rejected_frames,
        (unsigned long)s_ball.stable_frame_count,
        BallPositionAction_StateName(s_ball.state),
        (unsigned int)s_ball.servo_pulse_us);
}

bool Task3BallSequence_Init(void)
{
    s_initialized = true;
    s_state = TASK3_STATE_IDLE;
    s_fault_detail = 0U;
    return true;
}

bool Task3BallSequence_Start(uint32_t start_timestamp_ms)
{
    BallPositionActionCommand_t command;

    if ((!s_initialized) ||
        ((s_state != TASK3_STATE_IDLE) &&
         (s_state != TASK3_STATE_FINISHED)))
    {
        s_fault_detail = 20U;
        return false;
    }

    Task3_MakeCommand(
        &command,
        TASK3_TARGET_O_X,
        TASK3_O_TOLERANCE_MM,
        TASK3_SETTLE_SPEED_MM_S,
        TASK3_O_STABLE_FRAMES,
        TASK3_O_TIMEOUT_MS);
    if (!BallPositionAction_Start(
            &command,
            start_timestamp_ms))
    {
        s_fault_detail =
            100U + BallPositionAction_GetFaultDetail();
        return false;
    }

    s_state = TASK3_STATE_WAIT_O;
    s_start_ms = start_timestamp_ms;
    s_motion_start_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_report_ms = start_timestamp_ms;
    s_fault_detail = 0U;
    s_motion_started = false;
    s_result_locked = false;

    (void)BSP_Debug_Printf(
        "T3,START=1,SEQ=%ld>%ld>%ld,LIMIT=%lu\r\n",
        (long)TASK3_TARGET_O_X,
        (long)TASK3_TARGET_POS5_X,
        (long)TASK3_TARGET_NEG5_X,
        (unsigned long)TASK3_RUN_LIMIT_MS);
    return true;
}

Task3BallSequenceResult_t Task3BallSequence_Update(
    uint32_t now_ms)
{
    BallPositionActionResult_t result;

    if ((!s_initialized) ||
        (s_state == TASK3_STATE_FAULT))
    {
        return TASK3_BALL_SEQUENCE_RESULT_FAULT;
    }
    if (s_state == TASK3_STATE_FINISHED)
    {
        return TASK3_BALL_SEQUENCE_RESULT_FINISHED;
    }
    if (s_state == TASK3_STATE_IDLE)
    {
        return Task3_Fault(1U);
    }

    result = BallPositionAction_Update(now_ms);
    if ((result == BALL_POSITION_ACTION_RESULT_FAULT) ||
        !BallPositionAction_GetStatus(&s_ball))
    {
        return Task3_Fault(
            100U + BallPositionAction_GetFaultDetail());
    }
    Task3_Report(now_ms);

    if (s_motion_started && !s_result_locked &&
        ((uint32_t)(now_ms - s_motion_start_ms) >=
         TASK3_RUN_LIMIT_MS))
    {
        return Task3_Fault(2U);
    }

    if ((s_state == TASK3_STATE_WAIT_O) &&
        (result == BALL_POSITION_ACTION_RESULT_REACHED))
    {
        s_motion_start_ms = now_ms;
        s_motion_started = true;
        if (!Task3_Retarget(
                TASK3_TARGET_POS5_X,
                TASK3_POS5_TOLERANCE_MM,
                INT16_MAX,
                TASK3_POS5_STABLE_FRAMES,
                TASK3_POS5_TIMEOUT_MS,
                now_ms))
        {
            return Task3_Fault(3U);
        }
        s_state = TASK3_STATE_TO_POS5;
        (void)BSP_Debug_Printf(
            "T3,O_READY=1,TIMER=START,TARGET=%ld\r\n",
            (long)TASK3_TARGET_POS5_X);
    }
    else if ((s_state == TASK3_STATE_TO_POS5) &&
             (result == BALL_POSITION_ACTION_RESULT_REACHED))
    {
        if (!Task3_Retarget(
                TASK3_TARGET_NEG5_X,
                TASK3_NEG5_TOLERANCE_MM,
                TASK3_SETTLE_SPEED_MM_S,
                TASK3_NEG5_STABLE_FRAMES,
                TASK3_NEG5_TIMEOUT_MS,
                now_ms))
        {
            return Task3_Fault(4U);
        }
        s_state = TASK3_STATE_TO_NEG5;
        (void)BSP_Debug_Printf(
            "T3,POS5_REACHED=1,TARGET=%ld\r\n",
            (long)TASK3_TARGET_NEG5_X);
    }
    else if ((s_state == TASK3_STATE_TO_NEG5) &&
             (result == BALL_POSITION_ACTION_RESULT_REACHED))
    {
        s_result_elapsed_ms =
            now_ms - s_motion_start_ms;
        s_result_locked = true;
        s_state = TASK3_STATE_HOLD_NEG5;
        (void)BSP_Debug_Printf(
            "T3,FINISH=1,RESULT=PASS,MS=%lu,CX=%ld,ERRMM=%ld,VMM=%ld,STABLE=%lu,HOLD=1\r\n",
            (unsigned long)s_result_elapsed_ms,
            (long)s_ball.center_x,
            (long)s_ball.error_mm,
            (long)s_ball.velocity_mm_s,
            (unsigned long)s_ball.stable_frame_count);
        return TASK3_BALL_SEQUENCE_RESULT_FINISHED;
    }

    return (s_state == TASK3_STATE_HOLD_NEG5) ?
        TASK3_BALL_SEQUENCE_RESULT_FINISHED :
        TASK3_BALL_SEQUENCE_RESULT_RUNNING;
}

bool Task3BallSequence_Maintain(uint32_t now_ms)
{
    BallPositionActionResult_t result;

    if (s_state != TASK3_STATE_HOLD_NEG5)
    {
        return true;
    }
    result = BallPositionAction_Update(now_ms);
    if (result == BALL_POSITION_ACTION_RESULT_FAULT)
    {
        s_fault_detail =
            100U + BallPositionAction_GetFaultDetail();
        s_state = TASK3_STATE_FAULT;
        return false;
    }
    return BallPositionAction_GetStatus(&s_ball);
}

bool Task3BallSequence_RequestStop(void)
{
    if (!s_initialized)
    {
        return false;
    }
    if (s_state == TASK3_STATE_HOLD_NEG5)
    {
        return true;
    }
    if (s_state == TASK3_STATE_FINISHED)
    {
        return true;
    }
    if ((s_state == TASK3_STATE_IDLE) ||
        (s_state == TASK3_STATE_FAULT))
    {
        return false;
    }

    BallPositionAction_Stop();
    s_state = TASK3_STATE_FINISHED;
    return true;
}

bool Task3BallSequence_IsStopped(void)
{
    return (s_state == TASK3_STATE_IDLE) ||
           (s_state == TASK3_STATE_HOLD_NEG5) ||
           (s_state == TASK3_STATE_FINISHED) ||
           (s_state == TASK3_STATE_FAULT);
}

void Task3BallSequence_ForceSafeStop(void)
{
    BallPositionAction_ForceSafeStop();
    s_state = TASK3_STATE_IDLE;
}

bool Task3BallSequence_GetElapsedMs(
    uint32_t now_ms,
    uint32_t *elapsed_ms)
{
    if ((!s_initialized) || (elapsed_ms == 0) ||
        (s_state == TASK3_STATE_IDLE))
    {
        return false;
    }
    if (s_result_locked)
    {
        *elapsed_ms = s_result_elapsed_ms;
    }
    else if (s_motion_started)
    {
        *elapsed_ms = now_ms - s_motion_start_ms;
    }
    else
    {
        *elapsed_ms = 0U;
    }
    return true;
}

const char *Task3BallSequence_GetPhaseText(void)
{
    switch (s_state)
    {
        case TASK3_STATE_WAIT_O:
            return "T3 WAIT O";
        case TASK3_STATE_TO_POS5:
            return "T3 TO +5";
        case TASK3_STATE_TO_NEG5:
            return "T3 TO -5";
        case TASK3_STATE_HOLD_NEG5:
            return "T3 HOLD -5";
        case TASK3_STATE_FINISHED:
            return "T3 FINISHED";
        case TASK3_STATE_FAULT:
            return "T3 FAULT";
        case TASK3_STATE_IDLE:
        default:
            return "T3 READY";
    }
}

uint32_t Task3BallSequence_GetFaultDetail(void)
{
    return s_fault_detail;
}
