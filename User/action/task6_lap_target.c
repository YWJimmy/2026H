#include "task6_lap_target.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "distance_tracker.h"
#include "line_follow.h"
#include "line_follow_control.h"
#include "line_follow_control_config.h"
#include "line_sensor.h"
#include "task4_main_ball.h"
#include "task6_lap_target_config.h"
#include "task_menu_ui.h"
#include "vision.h"

typedef enum
{
    TASK6_STATE_IDLE = 0,
    TASK6_STATE_CAPTURE_TARGET,
    TASK6_STATE_START_HOLD,
    TASK6_STATE_START_RAMP,
    TASK6_STATE_LAP_RUNNING,
    TASK6_STATE_FIND_A_LINE,
    TASK6_STATE_POST_LINE_RUN,
    TASK6_STATE_STOPPING,
    TASK6_STATE_STOP_SETTLE,
    TASK6_STATE_USER_STOPPING,
    TASK6_STATE_FINISHED,
    TASK6_STATE_FAULT
} Task6State_t;

static bool s_initialized = false;
static Task6State_t s_state = TASK6_STATE_IDLE;
static uint32_t s_start_ms = 0U;
static uint32_t s_lap_elapsed_ms = 0U;
static uint32_t s_line_detect_ms = 0U;
static uint32_t s_start_hold_ms = 0U;
static uint32_t s_start_ramp_ms = 0U;
static uint32_t s_stop_start_ms = 0U;
static uint32_t s_stop_settle_ms = 0U;
static uint32_t s_last_update_ms = 0U;
static uint32_t s_last_report_ms = 0U;
static uint32_t s_fault_detail = 0U;
static int32_t s_line_center_mm = 0;
static bool s_lap_time_locked = false;
static bool s_time_pass = false;
static int32_t s_captured_target_x = 0;
static int32_t s_captured_target_mm = 0;
static bool s_ball_pass_at_a = false;
static uint32_t s_ball_max_error_at_a_mm = 0U;
static bool s_result_pass = false;
static uint8_t s_stop_speed_stage = 0U;
static bool s_has_fresh_sensor_frame = false;
static uint32_t s_last_sensor_sequence = 0U;
static uint8_t s_last_sensor_valid_mask = 0U;
static uint8_t s_line_window_bits = 0U;
static uint8_t s_line_window_total = 0U;
static uint8_t s_line_window_matched = 0U;
static LineSensorFrame_t s_line_frame;
static LineFollowResult_t s_line_result;
static LineFollowControlStatus_t s_line_control;
static DistanceTrackerStatus_t s_distance;
static ChassisStatus_t s_chassis;
static VisionStatus_t s_vision;
static Task4MainBallStatus_t s_ball;

static int32_t Task6_GetPostLineDistanceMm(void);

static int32_t Task6_ProgressMilli(uint32_t elapsed_ms,
                                   uint32_t duration_ms)
{
    if ((duration_ms == 0U) || (elapsed_ms >= duration_ms))
    {
        return 1000;
    }
    return (int32_t)(((uint64_t)elapsed_ms * 1000ULL) /
                     (uint64_t)duration_ms);
}

static void Task6_ApplyBallTransient(uint32_t now_ms)
{
    Task4MainBallTransient_t mode =
        TASK4_MAIN_BALL_TRANSIENT_NONE;
    int32_t progress_milli = 0;

    switch (s_state)
    {
        case TASK6_STATE_START_HOLD:
            mode = TASK4_MAIN_BALL_TRANSIENT_START_HOLD;
            progress_milli = Task6_ProgressMilli(
                now_ms - s_start_hold_ms,
                TASK6_START_HOLD_MS);
            break;
        case TASK6_STATE_START_RAMP:
            mode = TASK4_MAIN_BALL_TRANSIENT_START_RAMP;
            progress_milli = Task6_ProgressMilli(
                now_ms - s_start_ramp_ms,
                TASK6_START_RAMP_MS);
            break;
        case TASK6_STATE_POST_LINE_RUN:
            mode = TASK4_MAIN_BALL_TRANSIENT_STOP_APPROACH;
            progress_milli = Task6_ProgressMilli(
                (uint32_t)Task6_GetPostLineDistanceMm(),
                TASK6_POST_LINE_DISTANCE_MM);
            break;
        case TASK6_STATE_STOPPING:
            mode = TASK4_MAIN_BALL_TRANSIENT_STOPPING;
            progress_milli = Task6_ProgressMilli(
                now_ms - s_stop_start_ms,
                1500U);
            break;
        case TASK6_STATE_STOP_SETTLE:
            mode = TASK4_MAIN_BALL_TRANSIENT_STOP_SETTLE;
            progress_milli = Task6_ProgressMilli(
                now_ms - s_stop_settle_ms,
                TASK6_STOP_SETTLE_MS);
            break;
        default:
            break;
    }

    Task4MainBall_SetTransient(mode, progress_milli);
}

static Task6LapTargetResult_t Task6_Fault(uint32_t detail)
{
    s_fault_detail = detail;
    s_state = TASK6_STATE_FAULT;
    LineFollowControl_Stop(LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
    Task4MainBall_Stop();
    Vision_Stop();
    return TASK6_LAP_TARGET_RESULT_FAULT;
}

static bool Task6_UpdateBall(void)
{
    Vision_Update();
    if (!Vision_GetStatus(&s_vision) ||
        !Task4MainBall_Update(&s_vision))
    {
        return false;
    }
    return Task4MainBall_GetStatus(&s_ball);
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
    int32_t value = s_distance.center_signed_mm - s_line_center_mm;
    return (value > 0) ? value : 0;
}

static void Task6_UpdateOled(uint32_t now_ms)
{
    TaskMenuUi_SetRunningElapsedMs(
        s_lap_time_locked ? s_lap_elapsed_ms : (now_ms - s_start_ms),
        !s_lap_time_locked);
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
        "T6,ST=%u,MS=%lu,LAP=%lu,LOCK=%u,DIST=%lu,POST=%ld/%lu,"
        "MASK=0x%02X,CMD=%ld/%ld\r\n",
        (unsigned int)s_state,
        (unsigned long)(now_ms - s_start_ms),
        (unsigned long)(s_lap_time_locked ?
            s_lap_elapsed_ms : (now_ms - s_start_ms)),
        s_lap_time_locked ? 1U : 0U,
        (unsigned long)s_distance.traveled_mm,
        (long)(s_lap_time_locked ? Task6_GetPostLineDistanceMm() : 0),
        (unsigned long)TASK6_POST_LINE_DISTANCE_MM,
        (unsigned int)s_line_control.black_mask,
        (long)s_line_control.left_target_mm_s,
        (long)s_line_control.right_target_mm_s);
    (void)BSP_Debug_Printf(
        "T6BALL,CX=%ld,XMM=%ld,TGT=%ld,ERRMM=%ld,VF=%ld,MODE=%s,"
        "PULSE=%u,FF=%ld,MAXMM=%lu,IN1CM=%u,VIOL=%u,V=%u\r\n",
        (long)s_ball.center_x,
        (long)s_ball.position_mm,
        (long)s_ball.target_mm,
        (long)s_ball.error_mm,
        (long)s_ball.filtered_velocity_px_s,
        Task4MainBall_ModeName(s_ball.mode),
        (unsigned int)s_ball.servo_pulse_us,
        (long)s_ball.acceleration_feedforward_us,
        (unsigned long)s_ball.max_abs_error_mm,
        s_ball.within_one_cm ? 1U : 0U,
        s_ball.one_cm_violation_latched ? 1U : 0U,
        s_ball.vision_valid ? 1U : 0U);
    (void)BSP_Debug_Printf(
        "T6GAIN,SCH=%ld,XSCH=%ld,PG=%ld,VG=%ld,PRED=%ld,DB=%ld,"
        "FAST=%ld,REC=%ld,CB=%u,TP=%s,TPR=%ld\r\n",
        (long)s_ball.target_schedule_milli,
        (long)s_ball.extreme_schedule_milli,
        (long)s_ball.position_gain_milli,
        (long)s_ball.velocity_gain_milli,
        (long)s_ball.prediction_horizon_ms,
        (long)s_ball.position_deadband_px,
        (long)s_ball.fast_error_threshold_px,
        (long)s_ball.recover_error_threshold_px,
        (unsigned int)s_ball.cross_brake_frames,
        Task4MainBall_TransientName(s_ball.transient_mode),
        (long)s_ball.transient_progress_milli);
    (void)BSP_Debug_Printf(
        "T6MOTION,VFWD=%ld,TURN=%ld,AFF=%ld,CFF=%ld\r\n",
        (long)s_ball.chassis_forward_speed_mm_s,
        (long)s_ball.chassis_turn_command_mm_s,
        (long)s_ball.acceleration_feedforward_us,
        (long)s_ball.centrifugal_feedforward_us);
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
    if (!Task4MainBall_InitCaptureCurrent())
    {
        Vision_Stop();
        s_fault_detail = 8U;
        return false;
    }

    s_state = TASK6_STATE_CAPTURE_TARGET;
    s_start_ms = start_timestamp_ms;
    s_lap_elapsed_ms = 0U;
    s_line_detect_ms = 0U;
    s_start_hold_ms = 0U;
    s_start_ramp_ms = 0U;
    s_stop_start_ms = 0U;
    s_stop_settle_ms = 0U;
    s_last_update_ms = start_timestamp_ms;
    s_last_report_ms = start_timestamp_ms;
    s_fault_detail = 0U;
    s_line_center_mm = 0;
    s_lap_time_locked = false;
    s_time_pass = false;
    s_ball_pass_at_a = false;
    s_ball_max_error_at_a_mm = 0U;
    s_result_pass = false;
    s_stop_speed_stage = 0U;
    s_captured_target_x = 0;
    s_captured_target_mm = 0;
    s_has_fresh_sensor_frame = false;
    s_last_sensor_sequence = 0U;
    s_last_sensor_valid_mask = 0U;
    s_line_window_bits = 0U;
    s_line_window_total = 0U;
    s_line_window_matched = 0U;
    TaskMenuUi_SetRunningElapsedMs(0U, true);
    (void)BSP_Debug_Printf(
        "T6,START=1,CTRL=T4_PREDICTIVE,CAPTURE=3_STABLE,"
        "HOLD=%lu,RAMP=%lu,SEARCH_AFTER=%lu,POST=%lu,LIMIT=%lu\r\n",
        (unsigned long)TASK6_START_HOLD_MS,
        (unsigned long)TASK6_START_RAMP_MS,
        (unsigned long)TASK6_A_LINE_SEARCH_DISTANCE_MM,
        (unsigned long)TASK6_POST_LINE_DISTANCE_MM,
        (unsigned long)TASK6_LAP_TIMEOUT_MS);
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
    Task6_ApplyBallTransient(now_ms);
    if ((s_state != TASK6_STATE_USER_STOPPING) &&
        !Task6_UpdateBall())
    {
        return Task6_Fault(10U);
    }
    if (s_state == TASK6_STATE_CAPTURE_TARGET)
    {
        Task6_UpdateOled(now_ms);
        if (Task4MainBall_IsTargetLocked())
        {
            s_captured_target_x = s_ball.target_x;
            s_captured_target_mm = s_ball.target_mm;
            s_start_hold_ms = now_ms;
            s_state = TASK6_STATE_START_HOLD;
            Task4MainBall_SetTransient(
                TASK4_MAIN_BALL_TRANSIENT_START_HOLD,
                0);
            (void)BSP_Debug_Printf(
                "T6,TARGET_LOCK=1,CX=%ld,XMM=%ld,MS=%lu,"
                "START_HOLD=%lu\r\n",
                (long)s_captured_target_x,
                (long)s_captured_target_mm,
                (unsigned long)(now_ms - s_start_ms),
                (unsigned long)TASK6_START_HOLD_MS);
        }
        else if ((uint32_t)(now_ms - s_start_ms) >=
                 TASK6_TARGET_CAPTURE_TIMEOUT_MS)
        {
            return Task6_Fault(18U);
        }
        Task6_Report(now_ms);
        return TASK6_LAP_TARGET_RESULT_RUNNING;
    }

    if (s_state == TASK6_STATE_START_HOLD)
    {
        Task6_UpdateOled(now_ms);
        if ((uint32_t)(now_ms - s_start_hold_ms) >=
            TASK6_START_HOLD_MS)
        {
            if (!LineFollowControl_SetBaseSpeedRangeMmps(
                    TASK6_START_CENTER_SPEED_MM_S,
                    TASK6_START_MIN_SPEED_MM_S) ||
                !LineFollowControl_Start() ||
                !LineFollowControl_SetBaseSpeedRangeMmps(
                    LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S,
                    LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S))
            {
                return Task6_Fault(6U);
            }
            s_start_ramp_ms = now_ms;
            s_state = TASK6_STATE_START_RAMP;
            Task4MainBall_SetTransient(
                TASK4_MAIN_BALL_TRANSIENT_START_RAMP,
                0);
            (void)BSP_Debug_Printf(
                "T6,START_RAMP=1,MS=%lu,SPD=%ld/%ld,"
                "TARGET=%ld/%ld,RAMP=%lu\r\n",
                (unsigned long)(now_ms - s_start_ms),
                (long)TASK6_START_CENTER_SPEED_MM_S,
                (long)TASK6_START_MIN_SPEED_MM_S,
                (long)LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S,
                (long)LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S,
                (unsigned long)TASK6_START_RAMP_MS);
        }
        Task6_Report(now_ms);
        return TASK6_LAP_TARGET_RESULT_RUNNING;
    }

    if ((s_state != TASK6_STATE_STOPPING) &&
        (s_state != TASK6_STATE_STOP_SETTLE) &&
        (s_state != TASK6_STATE_USER_STOPPING) &&
        !Task6_UpdateLine(&has_new_line_result))
    {
        return Task6_Fault(11U);
    }
    LineFollowControl_Process();
    if (!DistanceTracker_GetStatus(&s_distance) ||
        !LineFollowControl_GetStatus(&s_line_control) ||
        !Chassis_GetStatus(&s_chassis))
    {
        return Task6_Fault(12U);
    }
    Task6_UpdateOled(now_ms);
    Task6_Report(now_ms);

    if ((s_state == TASK6_STATE_START_RAMP) &&
        ((uint32_t)(now_ms - s_start_ramp_ms) >=
         TASK6_START_RAMP_MS))
    {
        s_state = TASK6_STATE_LAP_RUNNING;
        Task4MainBall_SetTransient(
            TASK4_MAIN_BALL_TRANSIENT_NONE,
            0);
        (void)BSP_Debug_Printf(
            "T6,START_RAMP_DONE=1,MS=%lu,DIST=%lu\r\n",
            (unsigned long)(now_ms - s_start_ms),
            (unsigned long)s_distance.traveled_mm);
    }

    if (s_state == TASK6_STATE_USER_STOPPING)
    {
        if (!LineFollowControl_IsStopping() &&
            Chassis_IsMotionStopped())
        {
            s_result_pass = false;
            s_state = TASK6_STATE_FINISHED;
            TaskMenuUi_SetFinishedResult(false,
                s_lap_time_locked ? s_lap_elapsed_ms :
                    (now_ms - s_start_ms));
            return TASK6_LAP_TARGET_RESULT_FINISHED;
        }
        if ((uint32_t)(now_ms - s_stop_start_ms) >=
            TASK6_STOP_TIMEOUT_MS)
        {
            return Task6_Fault(13U);
        }
        return TASK6_LAP_TARGET_RESULT_RUNNING;
    }

    if (s_state == TASK6_STATE_STOPPING)
    {
        if (!LineFollowControl_IsStopping() &&
            Chassis_IsMotionStopped())
        {
            s_stop_settle_ms = now_ms;
            s_state = TASK6_STATE_STOP_SETTLE;
            Task4MainBall_SetTransient(
                TASK4_MAIN_BALL_TRANSIENT_STOP_SETTLE,
                0);
            (void)BSP_Debug_Printf(
                "T6,STOP_SETTLE=1,MS=%lu,ERRMM=%ld,VF=%ld\r\n",
                (unsigned long)(now_ms - s_start_ms),
                (long)s_ball.error_mm,
                (long)s_ball.filtered_velocity_px_s);
            return TASK6_LAP_TARGET_RESULT_RUNNING;
        }
        if ((uint32_t)(now_ms - s_stop_start_ms) >=
            TASK6_STOP_TIMEOUT_MS)
        {
            return Task6_Fault(13U);
        }
        return TASK6_LAP_TARGET_RESULT_RUNNING;
    }

    if (s_state == TASK6_STATE_STOP_SETTLE)
    {
        if ((uint32_t)(now_ms - s_stop_settle_ms) <
            TASK6_STOP_SETTLE_MS)
        {
            return TASK6_LAP_TARGET_RESULT_RUNNING;
        }

        s_state = TASK6_STATE_FINISHED;
        (void)BSP_Debug_Printf(
            "T6,RESULT=%s,MS=%lu,LAP=%lu,DIST=%lu,BALL_OK=%u,"
            "MAX_ERR_MM=%lu,FINAL_ERR_MM=%ld\r\n",
            s_result_pass ? "PASS" : "FAIL",
            (unsigned long)(now_ms - s_start_ms),
            (unsigned long)(s_lap_time_locked ?
                s_lap_elapsed_ms : (now_ms - s_start_ms)),
            (unsigned long)s_distance.traveled_mm,
            s_ball_pass_at_a ? 1U : 0U,
            (unsigned long)s_ball_max_error_at_a_mm,
            (long)s_ball.error_mm);
        TaskMenuUi_SetFinishedResult(
            s_result_pass,
            s_lap_time_locked ? s_lap_elapsed_ms :
                (now_ms - s_start_ms));
        return TASK6_LAP_TARGET_RESULT_FINISHED;
    }

    if (!s_lap_time_locked &&
        ((uint32_t)(now_ms - s_start_ms) > TASK6_LAP_TIMEOUT_MS))
    {
        return Task6_Fault(14U);
    }
    if (s_lap_time_locked &&
        ((uint32_t)(now_ms - s_line_detect_ms) >=
         TASK6_POST_LINE_TIMEOUT_MS))
    {
        return Task6_Fault(15U);
    }
    if (!LineFollowControl_IsRunning())
    {
        return Task6_Fault(16U);
    }
    if ((s_state == TASK6_STATE_LAP_RUNNING) &&
        (s_distance.traveled_mm >= TASK6_A_LINE_SEARCH_DISTANCE_MM))
    {
        s_state = TASK6_STATE_FIND_A_LINE;
        s_line_window_bits = 0U;
        s_line_window_total = 0U;
        s_line_window_matched = 0U;
        (void)BSP_Debug_Printf(
            "T6,FIND_A=1,DIST=%lu\r\n",
            (unsigned long)s_distance.traveled_mm);
    }
    if ((s_state == TASK6_STATE_FIND_A_LINE) && has_new_line_result)
    {
        bool matched = Task6_IsParkingLine(s_line_result.black_mask);
        Task6_UpdateLineWindow(matched);
        if ((s_line_window_total >= TASK6_A_LINE_WINDOW_FRAMES) &&
            (s_line_window_matched >= TASK6_A_LINE_REQUIRED_FRAMES))
        {
            s_lap_elapsed_ms = now_ms - s_start_ms;
            s_lap_time_locked = true;
            s_time_pass = (s_lap_elapsed_ms <= TASK6_LAP_TIMEOUT_MS);
            s_ball_pass_at_a = !s_ball.one_cm_violation_latched;
            s_ball_max_error_at_a_mm = s_ball.max_abs_error_mm;
            s_result_pass = s_time_pass && s_ball_pass_at_a;
            s_line_detect_ms = now_ms;
            s_line_center_mm = s_distance.center_signed_mm;
            if (!LineFollowControl_SetBaseSpeedRangeMmps(
                    TASK6_STOP_APPROACH_CENTER_SPEED_MM_S,
                    TASK6_STOP_APPROACH_MIN_SPEED_MM_S))
            {
                return Task6_Fault(19U);
            }
            s_stop_speed_stage = 1U;
            s_state = TASK6_STATE_POST_LINE_RUN;
            Task4MainBall_SetTransient(
                TASK4_MAIN_BALL_TRANSIENT_STOP_APPROACH,
                0);
            Task6_UpdateOled(now_ms);
            (void)BSP_Debug_Printf(
                "T6,A_LINE=1,LAP=%lu,TIME_OK=%u,BALL_OK=%u,"
                "MAX_ERR_MM=%lu,DIST=%lu,POST=%lu,SPD=%ld/%ld\r\n",
                (unsigned long)s_lap_elapsed_ms,
                s_time_pass ? 1U : 0U,
                s_ball_pass_at_a ? 1U : 0U,
                (unsigned long)s_ball_max_error_at_a_mm,
                (unsigned long)s_distance.traveled_mm,
                (unsigned long)TASK6_POST_LINE_DISTANCE_MM,
                (long)TASK6_STOP_APPROACH_CENTER_SPEED_MM_S,
                (long)TASK6_STOP_APPROACH_MIN_SPEED_MM_S);
        }
    }
    if ((s_state == TASK6_STATE_POST_LINE_RUN) &&
        (s_stop_speed_stage < 2U) &&
        (Task6_GetPostLineDistanceMm() >=
         (int32_t)TASK6_STOP_FINAL_STAGE_DISTANCE_MM))
    {
        if (!LineFollowControl_SetBaseSpeedRangeMmps(
                TASK6_STOP_FINAL_CENTER_SPEED_MM_S,
                TASK6_STOP_FINAL_MIN_SPEED_MM_S))
        {
            return Task6_Fault(21U);
        }
        s_stop_speed_stage = 2U;
        (void)BSP_Debug_Printf(
            "T6,STOP_FINAL_STAGE=1,POST=%ld,SPD=%ld/%ld\r\n",
            (long)Task6_GetPostLineDistanceMm(),
            (long)TASK6_STOP_FINAL_CENTER_SPEED_MM_S,
            (long)TASK6_STOP_FINAL_MIN_SPEED_MM_S);
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
            return Task6_Fault(17U);
        }
        s_stop_start_ms = now_ms;
        s_state = TASK6_STATE_STOPPING;
        Task4MainBall_SetTransient(
            TASK4_MAIN_BALL_TRANSIENT_STOPPING,
            0);
        (void)BSP_Debug_Printf(
            "T6,STOP_PROFILE=1,DECEL=%ld,JERK=%ld,"
            "ENTRY_SPD=%ld/%ld\r\n",
            (long)TASK6_STOP_DECEL_MM_S2,
            (long)TASK6_STOP_JERK_MM_S3,
            (long)s_line_control.left_target_mm_s,
            (long)s_line_control.right_target_mm_s);
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
        Task4MainBall_Stop();
        Vision_Stop();
        return true;
    }
    if (s_state == TASK6_STATE_USER_STOPPING)
    {
        return true;
    }
    if ((s_state != TASK6_STATE_CAPTURE_TARGET) &&
        (s_state != TASK6_STATE_START_HOLD) &&
        (s_state != TASK6_STATE_START_RAMP) &&
        (s_state != TASK6_STATE_LAP_RUNNING) &&
        (s_state != TASK6_STATE_FIND_A_LINE) &&
        (s_state != TASK6_STATE_POST_LINE_RUN) &&
        (s_state != TASK6_STATE_STOPPING) &&
        (s_state != TASK6_STATE_STOP_SETTLE))
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
    s_result_pass = false;
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
    Task4MainBall_Stop();
    Vision_Stop();
    s_state = TASK6_STATE_IDLE;
}

bool Task6LapTarget_GetElapsedMs(uint32_t now_ms, uint32_t *elapsed_ms)
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
        case TASK6_STATE_CAPTURE_TARGET: return "T6 CAPTURE TARGET";
        case TASK6_STATE_START_HOLD: return "T6 START HOLD";
        case TASK6_STATE_START_RAMP: return "T6 START RAMP";
        case TASK6_STATE_LAP_RUNNING: return "T6 LAP TARGET";
        case TASK6_STATE_FIND_A_LINE: return "T6 FIND A LINE";
        case TASK6_STATE_POST_LINE_RUN: return "T6 A TIME LOCK";
        case TASK6_STATE_STOPPING: return s_result_pass ? "T6 PASS STOP" : "T6 FAIL STOP";
        case TASK6_STATE_STOP_SETTLE: return "T6 STOP SETTLE";
        case TASK6_STATE_USER_STOPPING: return "T6 USER STOP";
        case TASK6_STATE_FINISHED: return s_result_pass ? "T6 PASS" : "T6 FAIL";
        case TASK6_STATE_FAULT: return "T6 FAULT";
        case TASK6_STATE_IDLE:
        default: return "T6 READY";
    }
}

uint32_t Task6LapTarget_GetFaultDetail(void)
{
    return s_fault_detail;
}
