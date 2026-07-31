#include "task4_ab_hold.h"

#include "ball_balance_control.h"
#include "bsp_debug_uart.h"
#include "chassis.h"
#include "distance_tracker.h"
#include "line_follow.h"
#include "line_follow_control.h"
#include "line_follow_control_config.h"
#include "line_sensor.h"
#include "task4_ab_hold_config.h"
#include "vision.h"

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

static bool s_initialized = false;
static Task4State_t s_state = TASK4_STATE_IDLE;
static uint32_t s_start_ms = 0U;
static uint32_t s_ab_elapsed_ms = 0U;
static uint32_t s_last_report_ms = 0U;
static uint32_t s_fault_detail = 0U;
static bool s_ab_time_locked = false;
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
static BallBalanceControlStatus_t s_ball;

static Task4AbHoldResult_t Task4_Fault(uint32_t detail)
{
    s_fault_detail = detail;
    s_state = TASK4_STATE_FAULT;
    LineFollowControl_Stop(
        LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
    BallBalanceControl_Stop();
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
    if (!BallBalanceControl_Update(&s_vision))
    {
        return false;
    }
    return BallBalanceControl_GetStatus(&s_ball);
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
        "T4,ST=%u,MS=%lu,AB=%lu,LOCK=%u,DIST=%lu,B=%ld,CMD=%ld/%ld\r\n",
        (unsigned int)s_state,
        (unsigned long)(now_ms - s_start_ms),
        (unsigned long)(s_ab_time_locked ?
            s_ab_elapsed_ms : (now_ms - s_start_ms)),
        s_ab_time_locked ? 1U : 0U,
        (unsigned long)s_distance.traveled_mm,
        (long)s_line_control.base_speed_mm_s,
        (long)s_line_control.left_target_mm_s,
        (long)s_line_control.right_target_mm_s);
    (void)BSP_Debug_Printf(
        "T4BALL,CX=%ld,TARGET=%ld,ERR=%ld,SPD=%ld,MODE=%s,PULSE=%u,V=%u\r\n",
        (long)s_ball.center_x,
        (long)s_ball.target_x,
        (long)s_ball.error,
        (long)s_ball.speed,
        BallBalanceControl_ModeName(s_ball.mode),
        (unsigned int)s_ball.servo_pulse_us,
        s_ball.vision_data_valid ? 1U : 0U);
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
    if (!LineFollowControl_Start())
    {
        s_fault_detail = 6U;
        return false;
    }
    if (!Vision_Init())
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
        s_fault_detail = 7U;
        return false;
    }
    if (!BallBalanceControl_Init() ||
        !BallBalanceControl_SetTargetX(
            TASK4_BALL_TARGET_X))
    {
        Vision_Stop();
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
        s_fault_detail = 8U;
        return false;
    }

    s_state = TASK4_STATE_RUNNING;
    s_start_ms = start_timestamp_ms;
    s_ab_elapsed_ms = 0U;
    s_last_report_ms = start_timestamp_ms;
    s_fault_detail = 0U;
    s_ab_time_locked = false;
    s_has_fresh_sensor_frame = false;
    s_last_sensor_sequence = 0U;
    s_last_sensor_valid_mask = 0U;

    (void)BSP_Debug_Printf(
        "T4,START=1,B_TIME=%lu,STOP_AT=%lu,BALL_X=%ld\r\n",
        (unsigned long)TASK4_B_TIME_DISTANCE_MM,
        (unsigned long)TASK4_STOP_START_DISTANCE_MM,
        (long)TASK4_BALL_TARGET_X);
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

    Task4_Report(now_ms);

    if ((s_state == TASK4_STATE_STOPPING) ||
        (s_state == TASK4_STATE_USER_STOPPING))
    {
        if (!LineFollowControl_IsStopping() &&
            Chassis_IsMotionStopped())
        {
            s_state = TASK4_STATE_FINISHED;
            (void)BSP_Debug_Printf(
                "T4,FINISH=1,MS=%lu,AB=%lu,DIST=%lu\r\n",
                (unsigned long)(now_ms - s_start_ms),
                (unsigned long)s_ab_elapsed_ms,
                (unsigned long)s_distance.traveled_mm);
            return TASK4_AB_HOLD_RESULT_FINISHED;
        }
        if ((uint32_t)(now_ms - s_start_ms) >=
            TASK4_RUN_TIMEOUT_MS)
        {
            return Task4_Fault(13U);
        }
        return TASK4_AB_HOLD_RESULT_RUNNING;
    }

    if ((uint32_t)(now_ms - s_start_ms) >=
        TASK4_RUN_TIMEOUT_MS)
    {
        return Task4_Fault(14U);
    }
    if (!LineFollowControl_IsRunning())
    {
        return Task4_Fault(15U);
    }

    if (!s_ab_time_locked &&
        (s_distance.traveled_mm >=
         TASK4_B_TIME_DISTANCE_MM))
    {
        s_ab_elapsed_ms = now_ms - s_start_ms;
        s_ab_time_locked = true;
        s_state = TASK4_STATE_B_TIMED;
        (void)BSP_Debug_Printf(
            "T4,B_REACHED=1,AB=%lu,DIST=%lu\r\n",
            (unsigned long)s_ab_elapsed_ms,
            (unsigned long)s_distance.traveled_mm);
    }

    if (s_distance.traveled_mm >=
        TASK4_STOP_START_DISTANCE_MM)
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
        BallBalanceControl_Stop();
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
        !LineFollowControl_RequestStop(
            LINE_FOLLOW_CONTROL_STOP_USER))
    {
        return false;
    }
    BallBalanceControl_Stop();
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
    if (BallBalanceControl_IsInitialized())
    {
        BallBalanceControl_Stop();
    }
    Vision_Stop();
    s_state = TASK4_STATE_IDLE;
}

bool Task4AbHold_GetElapsedMs(
    uint32_t now_ms,
    uint32_t *elapsed_ms)
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
            return "T4 TIME LOCKED";
        case TASK4_STATE_STOPPING:
            return "T4 STOPPING";
        case TASK4_STATE_USER_STOPPING:
            return "T4 USER STOP";
        case TASK4_STATE_FINISHED:
            return "T4 FINISHED";
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
