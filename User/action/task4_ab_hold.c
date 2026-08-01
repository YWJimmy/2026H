#include "task4_ab_hold.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "distance_tracker.h"
#include "line_follow.h"
#include "line_follow_control.h"
#include "line_follow_control_config.h"
#include "line_sensor.h"
#include "task4_ab_hold_config.h"
#include "task4_main_ball.h"
#include "task_menu_ui.h"
#include "vision.h"

#include <limits.h>

typedef enum
{
    TASK4_STATE_IDLE = 0,
    TASK4_STATE_RUNNING,
    TASK4_STATE_B_TIMED,
    TASK4_STATE_STOPPING,
    TASK4_STATE_USER_STOPPING,
    TASK4_STATE_FINISHED,
    TASK4_STATE_FAULT
} Task4State_t;

typedef enum
{
    TASK4_AB_LOCK_NONE = 0,
    TASK4_AB_LOCK_DISTANCE,
    TASK4_AB_LOCK_TURN
} Task4AbLockReason_t;

static bool s_initialized = false;
static Task4State_t s_state = TASK4_STATE_IDLE;
static uint32_t s_start_ms = 0U;
static uint32_t s_ab_elapsed_ms = 0U;
static uint32_t s_last_report_ms = 0U;
static uint32_t s_fault_detail = 0U;
static bool s_ab_time_locked = false;
static bool s_ab_time_pass = false;
static bool s_ball_pass_at_b = false;
static bool s_result_pass = false;
static Task4AbLockReason_t s_ab_lock_reason = TASK4_AB_LOCK_NONE;
static uint32_t s_turn_candidate_start_ms = 0U;
static uint32_t s_ball_max_error_at_b_mm = 0U;
static bool s_has_fresh_sensor_frame = false;
static uint32_t s_last_sensor_sequence = 0U;
static uint8_t s_last_sensor_valid_mask = 0U;

/* Keep the control snapshots out of the small main-loop stack. */
static LineSensorFrame_t s_line_frame;
static LineFollowResult_t s_line_result;
static LineFollowControlStatus_t s_line_control;
static DistanceTrackerStatus_t s_distance;
static ChassisStatus_t s_chassis;
static VisionStatus_t s_vision;
static Task4MainBallStatus_t s_ball;

static int32_t Task4_AbsI32(int32_t value)
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

static const char *Task4_LockReasonName(Task4AbLockReason_t reason)
{
    switch (reason)
    {
        case TASK4_AB_LOCK_DISTANCE:
            return "DIST";
        case TASK4_AB_LOCK_TURN:
            return "TURN";
        case TASK4_AB_LOCK_NONE:
        default:
            return "NONE";
    }
}

static bool Task4_IsTurnAtBConfirmed(uint32_t now_ms)
{
    int32_t turn_mm_s;

    if (s_distance.traveled_mm < TASK4_B_TURN_MIN_DISTANCE_MM)
    {
        s_turn_candidate_start_ms = 0U;
        return false;
    }

    turn_mm_s = Task4_AbsI32(s_line_control.correction_mm_s);
    if (turn_mm_s < TASK4_B_TURN_THRESHOLD_MM_S)
    {
        s_turn_candidate_start_ms = 0U;
        return false;
    }

    if (s_turn_candidate_start_ms == 0U)
    {
        s_turn_candidate_start_ms = now_ms;
        return false;
    }

    return ((uint32_t)(now_ms - s_turn_candidate_start_ms) >=
            TASK4_B_TURN_CONFIRM_MS);
}

static void Task4_UpdateOledTimer(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    elapsed_ms = s_ab_time_locked ?
        s_ab_elapsed_ms : (now_ms - s_start_ms);
    TaskMenuUi_SetRunningElapsedMs(
        elapsed_ms,
        !s_ab_time_locked);
}

static void Task4_LockAbTimer(
    uint32_t now_ms,
    Task4AbLockReason_t reason)
{
    if (s_ab_time_locked)
    {
        return;
    }

    s_ab_elapsed_ms = now_ms - s_start_ms;
    s_ab_time_locked = true;
    s_ab_lock_reason = reason;
    s_ab_time_pass = (s_ab_elapsed_ms <= TASK4_AB_TIME_LIMIT_MS);
    s_ball_pass_at_b = !s_ball.one_cm_violation_latched;
    s_ball_max_error_at_b_mm = s_ball.max_abs_error_mm;
    s_result_pass = s_ab_time_pass && s_ball_pass_at_b;
    s_state = TASK4_STATE_B_TIMED;
    Task4_UpdateOledTimer(now_ms);

    (void)BSP_Debug_Printf(
        "T4,B_REACHED=1,TRIG=%s,AB=%lu,TIME_OK=%u,BALL_OK=%u,"
        "MAX_ERR_MM=%lu,DIST=%lu,TURN=%ld\r\n",
        Task4_LockReasonName(reason),
        (unsigned long)s_ab_elapsed_ms,
        s_ab_time_pass ? 1U : 0U,
        s_ball_pass_at_b ? 1U : 0U,
        (unsigned long)s_ball_max_error_at_b_mm,
        (unsigned long)s_distance.traveled_mm,
        (long)s_line_control.correction_mm_s);
}

static Task4AbHoldResult_t Task4_Fault(uint32_t detail)
{
    s_fault_detail = detail;
    s_state = TASK4_STATE_FAULT;
    LineFollowControl_Stop(LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
    Task4MainBall_Stop();
    Vision_Stop();
    return TASK4_AB_HOLD_RESULT_FAULT;
}

static bool Task4_UpdateBall(void)
{
    Vision_Update();
    if (!Vision_GetStatus(&s_vision))
    {
        return false;
    }
    if (!Task4MainBall_Update(&s_vision))
    {
        return false;
    }
    return Task4MainBall_GetStatus(&s_ball);
}

static bool Task4_UpdateLine(void)
{
    bool has_new_frame;

    if (!LineSensor_GetFrame(&s_line_frame))
    {
        return true;
    }

    has_new_frame =
        (s_line_frame.sequence != s_last_sensor_sequence) ||
        (s_line_frame.valid_mask != s_last_sensor_valid_mask);
    if (!has_new_frame)
    {
        return true;
    }

    s_last_sensor_sequence = s_line_frame.sequence;
    s_last_sensor_valid_mask = s_line_frame.valid_mask;
    if (s_line_frame.valid_mask == 0xFFU)
    {
        s_has_fresh_sensor_frame = true;
    }
    if (!s_has_fresh_sensor_frame)
    {
        return true;
    }
    if (!LineFollow_Update(&s_line_frame) ||
        !LineFollow_GetResult(&s_line_result))
    {
        return false;
    }

    return LineFollowControl_Submit(&s_line_result);
}

static void Task4_Report(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_report_ms) <
        TASK4_REPORT_PERIOD_MS)
    {
        return;
    }

    s_last_report_ms = now_ms;
    (void)BSP_Debug_Printf(
        "T4,ST=%u,MS=%lu,AB=%lu,LOCK=%u,TRIG=%s,DIST=%lu,B=%ld,CMD=%ld/%ld\r\n",
        (unsigned int)s_state,
        (unsigned long)(now_ms - s_start_ms),
        (unsigned long)(s_ab_time_locked ?
            s_ab_elapsed_ms : (now_ms - s_start_ms)),
        s_ab_time_locked ? 1U : 0U,
        Task4_LockReasonName(s_ab_lock_reason),
        (unsigned long)s_distance.traveled_mm,
        (long)s_line_control.base_speed_mm_s,
        (long)s_line_control.left_target_mm_s,
        (long)s_line_control.right_target_mm_s);
    (void)BSP_Debug_Printf(
        "T4BALL,CX=%ld,ERR=%ld,PRED=%ld,VRAW=%ld,VF=%ld,"
        "MODE=%s,PULSE=%u,STEP=%u,BIAS=%ld,POS=%ld,DAMP=%ld,FF=%ld,"
        "CV=%ld,CA=%ld,MA=%ld,TURN=%ld,DYN=%u,LAUNCH=%u,"
        "MAXMM=%lu,IN1CM=%u,VIOL=%u,V=%u\r\n",
        (long)s_ball.center_x,
        (long)s_ball.error_px,
        (long)s_ball.predicted_error_px,
        (long)s_ball.raw_velocity_px_s,
        (long)s_ball.filtered_velocity_px_s,
        Task4MainBall_ModeName(s_ball.mode),
        (unsigned int)s_ball.servo_pulse_us,
        (unsigned int)s_ball.servo_step_limit_us,
        (long)s_ball.learned_bias_us,
        (long)s_ball.position_control_us,
        (long)s_ball.damping_control_us,
        (long)s_ball.acceleration_feedforward_us,
        (long)s_ball.chassis_forward_speed_mm_s,
        (long)s_ball.chassis_planned_accel_mm_s2,
        (long)s_ball.chassis_measured_accel_mm_s2,
        (long)s_ball.chassis_turn_command_mm_s,
        s_ball.dynamic_motion ? 1U : 0U,
        s_ball.launch_boost_active ? 1U : 0U,
        (unsigned long)s_ball.max_abs_error_mm,
        s_ball.within_one_cm ? 1U : 0U,
        s_ball.one_cm_violation_latched ? 1U : 0U,
        s_ball.vision_valid ? 1U : 0U);
}

bool Task4AbHold_Init(void)
{
    s_initialized = false;
    s_state = TASK4_STATE_IDLE;
    s_fault_detail = 0U;

    if (!LineSensor_IsRunning() && !LineSensor_Init())
    {
        s_fault_detail = 1U;
        return false;
    }
    if (!LineSensor_IsBackendConfigConfirmed())
    {
        s_fault_detail = 2U;
        return false;
    }
    if (!LineFollowControl_IsInitialized() &&
        !LineFollowControl_Init())
    {
        s_fault_detail = 3U;
        return false;
    }

    s_initialized = true;
    return true;
}

bool Task4AbHold_Start(uint32_t start_timestamp_ms)
{
    if ((!s_initialized) ||
        ((s_state != TASK4_STATE_IDLE) &&
         (s_state != TASK4_STATE_FINISHED)))
    {
        s_fault_detail = 20U;
        return false;
    }
    if (!LineSensor_IsRunning())
    {
        s_fault_detail = 4U;
        return false;
    }

    DistanceTracker_Reset();
    LineFollow_Init();
    LineSensor_DiscardFrame();
    if (!LineFollowControl_SetBaseSpeedRangeMmps(
            LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S,
            LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S))
    {
        s_fault_detail = 5U;
        return false;
    }
    /*
     * Arm vision and the servo before the chassis starts accelerating.
     * This removes the first-frame delay that previously allowed a large
     * launch disturbance before the ball controller became active.
     */
    if (!Vision_Init())
    {
        s_fault_detail = 7U;
        return false;
    }
    if (!Task4MainBall_Init())
    {
        Vision_Stop();
        s_fault_detail = 8U;
        return false;
    }
    if (!LineFollowControl_Start())
    {
        Task4MainBall_Stop();
        Vision_Stop();
        s_fault_detail = 6U;
        return false;
    }

    s_state = TASK4_STATE_RUNNING;
    s_start_ms = start_timestamp_ms;
    s_ab_elapsed_ms = 0U;
    s_last_report_ms = start_timestamp_ms;
    s_fault_detail = 0U;
    s_ab_time_locked = false;
    s_ab_time_pass = false;
    s_ball_pass_at_b = false;
    s_result_pass = false;
    s_ab_lock_reason = TASK4_AB_LOCK_NONE;
    s_turn_candidate_start_ms = 0U;
    s_ball_max_error_at_b_mm = 0U;
    s_has_fresh_sensor_frame = false;
    s_last_sensor_sequence = 0U;
    s_last_sensor_valid_mask = 0U;

    (void)BSP_Debug_Printf(
        "T4,START=1,CTRL=MAIN_LAUNCH_FF_OLED_V3,O_CX=653,TOL_PX=43,"
        "B_TIME=%lu,STOP_AT=%lu,AB_LIMIT=%lu,"
        "TURN_GATE=%lu,TURN_TH=%ld,TURN_MS=%lu\r\n",
        (unsigned long)TASK4_B_TIME_DISTANCE_MM,
        (unsigned long)TASK4_STOP_START_DISTANCE_MM,
        (unsigned long)TASK4_AB_TIME_LIMIT_MS,
        (unsigned long)TASK4_B_TURN_MIN_DISTANCE_MM,
        (long)TASK4_B_TURN_THRESHOLD_MM_S,
        (unsigned long)TASK4_B_TURN_CONFIRM_MS);
    TaskMenuUi_SetRunningElapsedMs(0U, true);
    return true;
}

Task4AbHoldResult_t Task4AbHold_Update(uint32_t now_ms)
{
    if ((!s_initialized) || (s_state == TASK4_STATE_FAULT))
    {
        return TASK4_AB_HOLD_RESULT_FAULT;
    }
    if (s_state == TASK4_STATE_FINISHED)
    {
        return TASK4_AB_HOLD_RESULT_FINISHED;
    }
    if (s_state == TASK4_STATE_IDLE)
    {
        return Task4_Fault(9U);
    }

    if ((s_state != TASK4_STATE_USER_STOPPING) &&
        !Task4_UpdateBall())
    {
        return Task4_Fault(10U);
    }
    if ((s_state != TASK4_STATE_STOPPING) &&
        (s_state != TASK4_STATE_USER_STOPPING) &&
        !Task4_UpdateLine())
    {
        return Task4_Fault(11U);
    }

    LineFollowControl_Process();
    if (!DistanceTracker_GetStatus(&s_distance) ||
        !LineFollowControl_GetStatus(&s_line_control) ||
        !Chassis_GetStatus(&s_chassis))
    {
        return Task4_Fault(12U);
    }

    if (!s_ab_time_locked)
    {
        if (s_distance.traveled_mm >= TASK4_B_TIME_DISTANCE_MM)
        {
            Task4_LockAbTimer(now_ms, TASK4_AB_LOCK_DISTANCE);
        }
        else if (Task4_IsTurnAtBConfirmed(now_ms))
        {
            Task4_LockAbTimer(now_ms, TASK4_AB_LOCK_TURN);
        }
    }
    Task4_UpdateOledTimer(now_ms);
    Task4_Report(now_ms);

    if ((s_state == TASK4_STATE_STOPPING) ||
        (s_state == TASK4_STATE_USER_STOPPING))
    {
        if (!LineFollowControl_IsStopping() &&
            Chassis_IsMotionStopped())
        {
            s_state = TASK4_STATE_FINISHED;
            s_result_pass =
                s_ab_time_locked &&
                s_ab_time_pass &&
                s_ball_pass_at_b;
            (void)BSP_Debug_Printf(
                "T4,RESULT=%s,MS=%lu,AB=%lu,DIST=%lu,"
                "BALL_OK=%u,MAX_ERR_MM=%lu\r\n",
                s_result_pass ? "PASS" : "FAIL",
                (unsigned long)(now_ms - s_start_ms),
                (unsigned long)s_ab_elapsed_ms,
                (unsigned long)s_distance.traveled_mm,
                s_ball_pass_at_b ? 1U : 0U,
                (unsigned long)s_ball_max_error_at_b_mm);
            TaskMenuUi_SetFinishedResult(
                s_result_pass,
                s_ab_time_locked ? s_ab_elapsed_ms :
                    (now_ms - s_start_ms));
            return TASK4_AB_HOLD_RESULT_FINISHED;
        }
        if ((uint32_t)(now_ms - s_start_ms) >= TASK4_RUN_TIMEOUT_MS)
        {
            return Task4_Fault(13U);
        }
        return TASK4_AB_HOLD_RESULT_RUNNING;
    }

    if ((uint32_t)(now_ms - s_start_ms) >= TASK4_RUN_TIMEOUT_MS)
    {
        return Task4_Fault(14U);
    }
    if (!LineFollowControl_IsRunning())
    {
        return Task4_Fault(15U);
    }

    if (s_distance.traveled_mm >= TASK4_STOP_START_DISTANCE_MM)
    {
        if (!LineFollowControl_RequestStopWithDecel(
                LINE_FOLLOW_CONTROL_STOP_TASK_COMPLETE,
                TASK4_STOP_DECEL_MM_S2,
                TASK4_STOP_JERK_MM_S3))
        {
            return Task4_Fault(16U);
        }
        s_state = TASK4_STATE_STOPPING;
        (void)BSP_Debug_Printf(
            "T4,STOPPING=1,DIST=%lu,DEC=%ld,JERK=%ld\r\n",
            (unsigned long)s_distance.traveled_mm,
            (long)TASK4_STOP_DECEL_MM_S2,
            (long)TASK4_STOP_JERK_MM_S3);
    }

    return TASK4_AB_HOLD_RESULT_RUNNING;
}

bool Task4AbHold_RequestStop(void)
{
    if (!s_initialized)
    {
        return false;
    }
    if (s_state == TASK4_STATE_FINISHED)
    {
        LineFollowControl_Shutdown();
        Task4MainBall_Stop();
        Vision_Stop();
        return true;
    }
    if (s_state == TASK4_STATE_USER_STOPPING)
    {
        return true;
    }
    if ((s_state != TASK4_STATE_RUNNING) &&
        (s_state != TASK4_STATE_B_TIMED) &&
        (s_state != TASK4_STATE_STOPPING))
    {
        return false;
    }
    if (LineFollowControl_IsRunning() &&
        !LineFollowControl_RequestStop(LINE_FOLLOW_CONTROL_STOP_USER))
    {
        return false;
    }

    Task4MainBall_Stop();
    Vision_Stop();
    s_state = TASK4_STATE_USER_STOPPING;
    return true;
}

bool Task4AbHold_IsStopped(void)
{
    return (s_state == TASK4_STATE_FINISHED) ||
           (s_state == TASK4_STATE_IDLE) ||
           (s_state == TASK4_STATE_FAULT) ||
           Chassis_IsMotionStopped();
}

void Task4AbHold_ForceSafeStop(void)
{
    if (LineFollowControl_IsInitialized())
    {
        LineFollowControl_Shutdown();
    }
    if (Task4MainBall_IsInitialized())
    {
        Task4MainBall_Stop();
    }
    Vision_Stop();
    s_state = TASK4_STATE_IDLE;
}

bool Task4AbHold_GetElapsedMs(uint32_t now_ms, uint32_t *elapsed_ms)
{
    if ((!s_initialized) || (elapsed_ms == 0))
    {
        return false;
    }
    *elapsed_ms = s_ab_time_locked ?
        s_ab_elapsed_ms : (now_ms - s_start_ms);
    return (s_state != TASK4_STATE_IDLE);
}

const char *Task4AbHold_GetPhaseText(void)
{
    switch (s_state)
    {
        case TASK4_STATE_RUNNING:
            return "T4 TO B + HOLD";
        case TASK4_STATE_B_TIMED:
            return s_result_pass ? "T4 B PASS" : "T4 B FAIL";
        case TASK4_STATE_STOPPING:
            return s_result_pass ? "T4 PASS STOP" : "T4 FAIL STOP";
        case TASK4_STATE_USER_STOPPING:
            return "T4 USER STOP";
        case TASK4_STATE_FINISHED:
            return s_result_pass ? "T4 PASS" : "T4 FAIL";
        case TASK4_STATE_FAULT:
            return "T4 FAULT";
        case TASK4_STATE_IDLE:
        default:
            return "T4 READY";
    }
}

uint32_t Task4AbHold_GetFaultDetail(void)
{
    return s_fault_detail;
}
