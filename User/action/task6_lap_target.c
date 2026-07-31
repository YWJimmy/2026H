#include "task6_lap_target.h"

#include "ball_balance_control.h"
#include "bsp_debug_uart.h"
#include "chassis.h"
#include "distance_tracker.h"
#include "line_follow.h"
#include "line_follow_control.h"
#include "line_follow_control_config.h"
#include "line_sensor.h"
#include "task6_lap_target_config.h"
#include "vision.h"

typedef enum
{
    TASK6_STATE_IDLE = 0,
    TASK6_STATE_CAPTURE_TARGET,
    TASK6_STATE_LAP_RUNNING,
    TASK6_STATE_FIND_A_LINE,
    TASK6_STATE_POST_LINE_RUN,
    TASK6_STATE_STOPPING,
    TASK6_STATE_USER_STOPPING,
    TASK6_STATE_FINISHED,
    TASK6_STATE_FAULT
} Task6State_t;

static bool s_initialized = false;
static Task6State_t s_state = TASK6_STATE_IDLE;
static uint32_t s_start_ms = 0U;
static uint32_t s_lap_elapsed_ms = 0U;
static uint32_t s_line_detect_ms = 0U;
static uint32_t s_stop_start_ms = 0U;
static uint32_t s_last_update_ms = 0U;
static uint32_t s_last_report_ms = 0U;
static uint32_t s_fault_detail = 0U;
static int32_t s_ball_target_x = 0;
static int32_t s_line_center_mm = 0;
static bool s_lap_time_locked = false;
static bool s_has_fresh_sensor_frame = false;
static uint32_t s_last_sensor_sequence = 0U;
static uint8_t s_last_sensor_valid_mask = 0U;
static uint8_t s_line_window_bits = 0U;
static uint8_t s_line_window_total = 0U;
static uint8_t s_line_window_matched = 0U;

/* Keep the large snapshots out of the main-loop stack. */
static LineSensorFrame_t s_line_frame;
static LineFollowResult_t s_line_result;
static LineFollowControlStatus_t s_line_control;
static DistanceTrackerStatus_t s_distance;
static ChassisStatus_t s_chassis;
static VisionStatus_t s_vision;
static BallBalanceControlStatus_t s_ball;

static Task6LapTargetResult_t Task6_Fault(uint32_t detail)
{
    s_fault_detail = detail;
    s_state = TASK6_STATE_FAULT;
    LineFollowControl_Stop(
        LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
    BallBalanceControl_Stop();
    Vision_Stop();
    return TASK6_LAP_TARGET_RESULT_FAULT;
}

static bool Task6_UpdateBall(void)
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

static bool Task6_IsParkingLine(uint8_t black_mask)
{
    return (((black_mask & TASK6_A_LINE_ADC345_MASK) ==
             TASK6_A_LINE_ADC345_MASK) ||
            ((black_mask & TASK6_A_LINE_ADC456_MASK) ==
             TASK6_A_LINE_ADC456_MASK));
}

static void Task6_UpdateLineWindow(bool matched)
{
    if (s_line_window_total >= TASK6_A_LINE_WINDOW_FRAMES)
    {
        if ((s_line_window_bits & 0x10U) != 0U)
        {
            s_line_window_matched--;
        }
    }
    else
    {
        s_line_window_total++;
    }

    s_line_window_bits = (uint8_t)(
        ((s_line_window_bits << 1U) & 0x1FU) |
        (matched ? 1U : 0U));
    if (matched)
    {
        s_line_window_matched++;
    }
}

static bool Task6_UpdateLine(bool *has_new_result)
{
    bool has_new_frame;

    *has_new_result = false;
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
    *has_new_result = true;
    return LineFollowControl_Submit(&s_line_result);
}

static int32_t Task6_GetPostLineDistanceMm(void)
{
    int32_t distance_mm =
        s_distance.center_signed_mm - s_line_center_mm;

    return (distance_mm > 0) ? distance_mm : 0;
}

static void Task6_Report(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_report_ms) <
        TASK6_REPORT_PERIOD_MS)
    {
        return;
    }

    s_last_report_ms = now_ms;
    (void)BSP_Debug_Printf(
        "T6,ST=%u,MS=%lu,LAP=%lu,LOCK=%u,DIST=%lu,POST=%ld/%lu,MASK=0x%02X,B=%ld,CMD=%ld/%ld\r\n",
        (unsigned int)s_state,
        (unsigned long)(now_ms - s_start_ms),
        (unsigned long)(s_lap_time_locked ?
            s_lap_elapsed_ms : (now_ms - s_start_ms)),
        s_lap_time_locked ? 1U : 0U,
        (unsigned long)s_distance.traveled_mm,
        (long)(s_lap_time_locked ?
            Task6_GetPostLineDistanceMm() : 0),
        (unsigned long)TASK6_POST_LINE_DISTANCE_MM,
        (unsigned int)s_line_control.black_mask,
        (long)s_line_control.base_speed_mm_s,
        (long)s_line_control.left_target_mm_s,
        (long)s_line_control.right_target_mm_s);
    (void)BSP_Debug_Printf(
        "T6BALL,CX=%ld,TARGET=%ld,ERR=%ld,SPD=%ld,MODE=%s,PULSE=%u,V=%u\r\n",
        (long)s_ball.center_x,
        (long)s_ball.target_x,
        (long)s_ball.error,
        (long)s_ball.speed,
        BallBalanceControl_ModeName(s_ball.mode),
        (unsigned int)s_ball.servo_pulse_us,
        s_ball.vision_data_valid ? 1U : 0U);
}

bool Task6LapTarget_Init(void)
{
    s_initialized = false;
    s_state = TASK6_STATE_IDLE;
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

bool Task6LapTarget_Start(uint32_t start_timestamp_ms)
{
    if ((!s_initialized) ||
        ((s_state != TASK6_STATE_IDLE) &&
         (s_state != TASK6_STATE_FINISHED)))
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
    if (!Vision_Init())
    {
        s_fault_detail = 7U;
        return false;
    }
    if (!BallBalanceControl_Init())
    {
        Vision_Stop();
        s_fault_detail = 8U;
        return false;
    }

    s_state = TASK6_STATE_CAPTURE_TARGET;
    s_start_ms = start_timestamp_ms;
    s_lap_elapsed_ms = 0U;
    s_line_detect_ms = 0U;
    s_stop_start_ms = 0U;
    s_last_update_ms = start_timestamp_ms;
    s_last_report_ms = start_timestamp_ms;
    s_fault_detail = 0U;
    s_ball_target_x = 0;
    s_line_center_mm = 0;
    s_lap_time_locked = false;
    s_has_fresh_sensor_frame = false;
    s_last_sensor_sequence = 0U;
    s_last_sensor_valid_mask = 0U;
    s_line_window_bits = 0U;
    s_line_window_total = 0U;
    s_line_window_matched = 0U;

    (void)BSP_Debug_Printf(
        "T6,START=1,TARGET=AUTO,WAIT=%lu,SEARCH_AFTER=%lu,POST=%lu\r\n",
        (unsigned long)TASK6_TARGET_CAPTURE_TIMEOUT_MS,
        (unsigned long)TASK6_A_LINE_SEARCH_DISTANCE_MM,
        (unsigned long)TASK6_POST_LINE_DISTANCE_MM);
    return true;
}

Task6LapTargetResult_t Task6LapTarget_Update(uint32_t now_ms)
{
    bool has_new_line_result = false;

    s_last_update_ms = now_ms;
    if ((!s_initialized) || (s_state == TASK6_STATE_FAULT))
    {
        return TASK6_LAP_TARGET_RESULT_FAULT;
    }
    if (s_state == TASK6_STATE_FINISHED)
    {
        return TASK6_LAP_TARGET_RESULT_FINISHED;
    }
    if (s_state == TASK6_STATE_IDLE)
    {
        return Task6_Fault(9U);
    }

    if (s_state == TASK6_STATE_CAPTURE_TARGET)
    {
        Vision_Update();
        if (!Vision_GetStatus(&s_vision) ||
            !BallBalanceControl_GetStatus(&s_ball))
        {
            return Task6_Fault(10U);
        }

        if (s_vision.data_valid &&
            s_vision.has_frame &&
            s_vision.frame.found)
        {
            s_ball_target_x =
                (int32_t)s_vision.frame.center_x;
            if (!BallBalanceControl_SetTargetX(
                    s_ball_target_x) ||
                !BallBalanceControl_Update(&s_vision))
            {
                return Task6_Fault(11U);
            }
            if (!LineFollowControl_Start())
            {
                return Task6_Fault(12U);
            }
            s_state = TASK6_STATE_LAP_RUNNING;
            (void)BSP_Debug_Printf(
                "T6,TARGET_LOCK=1,X=%ld,MS=%lu,DRIVE=START\r\n",
                (long)s_ball_target_x,
                (unsigned long)(now_ms - s_start_ms));
        }
        else if ((uint32_t)(now_ms - s_start_ms) >=
                 TASK6_TARGET_CAPTURE_TIMEOUT_MS)
        {
            return Task6_Fault(13U);
        }

        LineFollowControl_Process();
        if (!DistanceTracker_GetStatus(&s_distance) ||
            !LineFollowControl_GetStatus(&s_line_control) ||
            !Chassis_GetStatus(&s_chassis))
        {
            return Task6_Fault(14U);
        }
        Task6_Report(now_ms);
        return TASK6_LAP_TARGET_RESULT_RUNNING;
    }

    if ((s_state != TASK6_STATE_USER_STOPPING) &&
        !Task6_UpdateBall())
    {
        return Task6_Fault(15U);
    }
    if ((s_state != TASK6_STATE_STOPPING) &&
        (s_state != TASK6_STATE_USER_STOPPING) &&
        !Task6_UpdateLine(&has_new_line_result))
    {
        return Task6_Fault(16U);
    }

    LineFollowControl_Process();
    if (!DistanceTracker_GetStatus(&s_distance) ||
        !LineFollowControl_GetStatus(&s_line_control) ||
        !Chassis_GetStatus(&s_chassis))
    {
        return Task6_Fault(17U);
    }
    Task6_Report(now_ms);

    if ((s_state == TASK6_STATE_STOPPING) ||
        (s_state == TASK6_STATE_USER_STOPPING))
    {
        if (!LineFollowControl_IsStopping() &&
            Chassis_IsMotionStopped())
        {
            s_state = TASK6_STATE_FINISHED;
            (void)BSP_Debug_Printf(
                "T6,FINISH=1,MS=%lu,LAP=%lu,DIST=%lu,POST=%ld,TARGET=%ld\r\n",
                (unsigned long)(now_ms - s_start_ms),
                (unsigned long)s_lap_elapsed_ms,
                (unsigned long)s_distance.traveled_mm,
                (long)(s_lap_time_locked ?
                    Task6_GetPostLineDistanceMm() : 0),
                (long)s_ball_target_x);
            return TASK6_LAP_TARGET_RESULT_FINISHED;
        }
        if ((uint32_t)(now_ms - s_stop_start_ms) >=
            TASK6_STOP_TIMEOUT_MS)
        {
            return Task6_Fault(18U);
        }
        return TASK6_LAP_TARGET_RESULT_RUNNING;
    }

    if (!s_lap_time_locked &&
        ((uint32_t)(now_ms - s_start_ms) >=
         TASK6_LAP_TIMEOUT_MS))
    {
        return Task6_Fault(19U);
    }
    if (s_lap_time_locked &&
        ((uint32_t)(now_ms - s_line_detect_ms) >=
         TASK6_POST_LINE_TIMEOUT_MS))
    {
        return Task6_Fault(21U);
    }
    if (!LineFollowControl_IsRunning())
    {
        return Task6_Fault(22U);
    }

    if ((s_state == TASK6_STATE_LAP_RUNNING) &&
        (s_distance.traveled_mm >=
         TASK6_A_LINE_SEARCH_DISTANCE_MM))
    {
        s_state = TASK6_STATE_FIND_A_LINE;
        s_line_window_bits = 0U;
        s_line_window_total = 0U;
        s_line_window_matched = 0U;
        (void)BSP_Debug_Printf(
            "T6,FIND_A=1,DIST=%lu,SPEED_KEEP=%ld/%ld\r\n",
            (unsigned long)s_distance.traveled_mm,
            (long)LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S,
            (long)LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S);
    }

    if ((s_state == TASK6_STATE_FIND_A_LINE) &&
        has_new_line_result)
    {
        bool matched = Task6_IsParkingLine(
            s_line_result.black_mask);

        Task6_UpdateLineWindow(matched);
        if (matched && (s_line_window_matched == 1U))
        {
            (void)BSP_Debug_Printf(
                "T6,A_CAND=1,DIST=%lu,MASK=0x%02X,N=%u\r\n",
                (unsigned long)s_distance.traveled_mm,
                (unsigned int)s_line_result.black_mask,
                (unsigned int)s_line_result.black_count);
        }

        if ((s_line_window_total >=
             TASK6_A_LINE_WINDOW_FRAMES) &&
            (s_line_window_matched >=
             TASK6_A_LINE_REQUIRED_FRAMES))
        {
            s_lap_elapsed_ms = now_ms - s_start_ms;
            s_lap_time_locked = true;
            s_line_detect_ms = now_ms;
            s_line_center_mm = s_distance.center_signed_mm;
            s_state = TASK6_STATE_POST_LINE_RUN;
            (void)BSP_Debug_Printf(
                "T6,A_LINE=1,LAP=%lu,DIST=%lu,C=%ld,HIT=%u/%u,POST=%lu\r\n",
                (unsigned long)s_lap_elapsed_ms,
                (unsigned long)s_distance.traveled_mm,
                (long)s_line_center_mm,
                (unsigned int)s_line_window_matched,
                (unsigned int)s_line_window_total,
                (unsigned long)TASK6_POST_LINE_DISTANCE_MM);
        }
    }

    if ((s_state == TASK6_STATE_POST_LINE_RUN) &&
        (Task6_GetPostLineDistanceMm() >=
         (int32_t)TASK6_POST_LINE_DISTANCE_MM))
    {
        if (!LineFollowControl_RequestStopWithDecel(
                LINE_FOLLOW_CONTROL_STOP_TASK_COMPLETE,
                TASK6_STOP_DECEL_MM_S2,
                TASK6_STOP_JERK_MM_S3))
        {
            return Task6_Fault(23U);
        }
        s_stop_start_ms = now_ms;
        s_state = TASK6_STATE_STOPPING;
        (void)BSP_Debug_Printf(
            "T6,STOPPING=1,DIST=%lu,POST=%ld,DEC=%ld,JERK=%ld\r\n",
            (unsigned long)s_distance.traveled_mm,
            (long)Task6_GetPostLineDistanceMm(),
            (long)TASK6_STOP_DECEL_MM_S2,
            (long)TASK6_STOP_JERK_MM_S3);
    }

    return TASK6_LAP_TARGET_RESULT_RUNNING;
}

bool Task6LapTarget_RequestStop(void)
{
    if (!s_initialized)
    {
        return false;
    }
    if (s_state == TASK6_STATE_FINISHED)
    {
        LineFollowControl_Shutdown();
        BallBalanceControl_Stop();
        Vision_Stop();
        return true;
    }
    if (s_state == TASK6_STATE_USER_STOPPING)
    {
        return true;
    }
    if ((s_state != TASK6_STATE_CAPTURE_TARGET) &&
        (s_state != TASK6_STATE_LAP_RUNNING) &&
        (s_state != TASK6_STATE_FIND_A_LINE) &&
        (s_state != TASK6_STATE_POST_LINE_RUN) &&
        (s_state != TASK6_STATE_STOPPING))
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
    s_stop_start_ms = s_last_update_ms;
    s_state = TASK6_STATE_USER_STOPPING;
    return true;
}

bool Task6LapTarget_IsStopped(void)
{
    return (s_state == TASK6_STATE_FINISHED) ||
           (s_state == TASK6_STATE_IDLE) ||
           (s_state == TASK6_STATE_FAULT) ||
           Chassis_IsMotionStopped();
}

void Task6LapTarget_ForceSafeStop(void)
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
    s_state = TASK6_STATE_IDLE;
}

bool Task6LapTarget_GetElapsedMs(
    uint32_t now_ms,
    uint32_t *elapsed_ms)
{
    if ((!s_initialized) || (elapsed_ms == 0))
    {
        return false;
    }
    *elapsed_ms = s_lap_time_locked ?
        s_lap_elapsed_ms : (now_ms - s_start_ms);
    return (s_state != TASK6_STATE_IDLE);
}

const char *Task6LapTarget_GetPhaseText(void)
{
    switch (s_state)
    {
        case TASK6_STATE_CAPTURE_TARGET:
            return "T6 CAPTURE BALL";
        case TASK6_STATE_LAP_RUNNING:
            return "T6 LAP + TARGET";
        case TASK6_STATE_FIND_A_LINE:
            return "T6 FIND A LINE";
        case TASK6_STATE_POST_LINE_RUN:
            return "T6 TIME + 300MM";
        case TASK6_STATE_STOPPING:
            return "T6 STOPPING";
        case TASK6_STATE_USER_STOPPING:
            return "T6 USER STOP";
        case TASK6_STATE_FINISHED:
            return "T6 FINISHED";
        case TASK6_STATE_FAULT:
            return "T6 FAULT";
        case TASK6_STATE_IDLE:
        default:
            return "T6 READY";
    }
}

uint32_t Task6LapTarget_GetFaultDetail(void)
{
    return s_fault_detail;
}
