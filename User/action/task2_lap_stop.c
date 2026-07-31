#include "task2_lap_stop.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "distance_tracker.h"
#include "line_follow.h"
#include "line_follow_control.h"
#include "line_follow_control_config.h"
#include "line_sensor.h"
#include "task2_lap_stop_config.h"

#include <limits.h>

typedef enum
{
    TASK2_STATE_IDLE = 0,
    TASK2_STATE_LAP_RUNNING,
    TASK2_STATE_PREDECEL,
    TASK2_STATE_POSITION_APPROACH,
    TASK2_STATE_DISTANCE_STOPPING,
    TASK2_STATE_USER_STOPPING,
    TASK2_STATE_FINISHED,
    TASK2_STATE_FAULT
} Task2State_t;

static bool s_initialized = false;
static Task2State_t s_state = TASK2_STATE_IDLE;
static uint32_t s_start_ms = 0U;
static uint32_t s_last_report_ms = 0U;
static uint32_t s_line_candidate_start_ms = 0U;
static bool s_line_candidate = false;
static uint8_t s_line_window_total = 0U;
static uint8_t s_line_window_matched = 0U;
static bool s_has_fresh_sensor_frame = false;
static uint32_t s_last_sensor_sequence = 0U;
static uint8_t s_last_sensor_valid_mask = 0U;
static int32_t s_line_center_mm = 0;
static int32_t s_stop_target_center_mm = 0;
static int32_t s_stop_offset_mm = 0;
static int32_t s_position_speed_mm_s = 0;
static uint32_t s_fault_detail = 0U;

/*
 * Task2LapStop_Update() is non-reentrant and runs only in the main loop.
 * Keep its large snapshots out of the 1 kB-class call stack: together these
 * structures occupy more than 400 bytes before nested control and printf
 * calls are counted.
 */
static LineSensorFrame_t s_frame_snapshot;
static LineFollowResult_t s_line_result_snapshot;
static LineFollowControlStatus_t s_control_snapshot;
static DistanceTrackerStatus_t s_distance_snapshot;
static ChassisStatus_t s_chassis_snapshot;

static int32_t Task2_AbsI32(int32_t value)
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

static int32_t Task2_ClampI32(
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

static Task2LapStopResult_t Task2_Fault(uint32_t detail)
{
    s_fault_detail = detail;
    s_state = TASK2_STATE_FAULT;
    LineFollowControl_Stop(
        LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
    return TASK2_LAP_STOP_RESULT_FAULT;
}

static int32_t Task2_GetPositionProgressMm(
    const DistanceTrackerStatus_t *distance)
{
    int32_t progress_mm;

    if ((s_state != TASK2_STATE_POSITION_APPROACH) &&
        (s_state != TASK2_STATE_DISTANCE_STOPPING))
    {
        return 0;
    }

    progress_mm = distance->center_signed_mm - s_line_center_mm;

    if (progress_mm < 0)
    {
        progress_mm = 0;
    }
    return progress_mm;
}

static int32_t Task2_GetPositionSpeedMmps(int32_t remaining_mm)
{
    int64_t speed =
        ((int64_t)remaining_mm *
         (int64_t)TASK2_POSITION_KP_Q10) >> 10;

    return Task2_ClampI32(
        (int32_t)speed,
        TASK2_POSITION_MIN_SPEED_MM_S,
        TASK2_POSITION_MAX_SPEED_MM_S);
}

static bool Task2_StartPositionApproach(
    const DistanceTrackerStatus_t *distance)
{
    s_line_center_mm = distance->center_signed_mm;
    s_stop_target_center_mm =
        s_line_center_mm +
        (int32_t)TASK2_SENSOR_FORWARD_OFFSET_MM;
    s_stop_offset_mm = 0;
    s_position_speed_mm_s =
        TASK2_POSITION_MAX_SPEED_MM_S;

    if (!LineFollowControl_SetBaseSpeedRangeMmps(
            s_position_speed_mm_s,
            s_position_speed_mm_s))
    {
        return false;
    }

    s_state = TASK2_STATE_POSITION_APPROACH;

    (void)BSP_Debug_Printf(
        "T2,A_LINE=1,DIST=%lu,MASK=0x%02X,N=%u,C=%ld,TARGET=%ld,HIT=%u/%u\r\n",
        (unsigned long)distance->traveled_mm,
        (unsigned int)s_line_result_snapshot.black_mask,
        (unsigned int)s_line_result_snapshot.black_count,
        (long)s_line_center_mm,
        (long)s_stop_target_center_mm,
        (unsigned int)s_line_window_matched,
        (unsigned int)s_line_window_total);
    return true;
}

static void Task2_Report(
    uint32_t now_ms,
    const DistanceTrackerStatus_t *distance,
    const LineFollowControlStatus_t *control,
    const ChassisStatus_t *chassis)
{
    if ((uint32_t)(now_ms - s_last_report_ms) <
        TASK2_REPORT_PERIOD_MS)
    {
        return;
    }

    s_last_report_ms = now_ms;
    (void)BSP_Debug_Printf(
        "T2,ST=%u,MS=%lu,DIST=%lu,SDIST=%ld,MASK=0x%02X,B=%ld,POS=%ld/%lu,OFF=%ld\r\n",
        (unsigned int)s_state,
        (unsigned long)(now_ms - s_start_ms),
        (unsigned long)distance->traveled_mm,
        (long)distance->center_signed_mm,
        (unsigned int)control->black_mask,
        (long)control->base_speed_mm_s,
        (long)Task2_GetPositionProgressMm(distance),
        (unsigned long)TASK2_SENSOR_FORWARD_OFFSET_MM,
        (long)s_stop_offset_mm);

    (void)BSP_Debug_Printf(
        "T2C,RE=%d,E=%d,DE=%d,CT=%ld,C=%ld,CMD=%ld/%ld\r\n",
        (int)control->raw_error,
        (int)control->error,
        (int)control->error_delta,
        (long)control->correction_target_mm_s,
        (long)control->correction_mm_s,
        (long)control->left_target_mm_s,
        (long)control->right_target_mm_s);

    (void)BSP_Debug_Printf(
        "T2W,T=%ld/%ld,MEA=%ld/%ld,DT=%u,OVR=%lu\r\n",
        (long)chassis->left_target_mm_s,
        (long)chassis->right_target_mm_s,
        (long)chassis->left_measured_mm_s,
        (long)chassis->right_measured_mm_s,
        (unsigned int)chassis->dt_ms,
        (unsigned long)chassis->timing_overrun_count);
}

bool Task2LapStop_Init(void)
{
    s_initialized = false;
    s_state = TASK2_STATE_IDLE;
    s_fault_detail = 0U;

    if (!LineSensor_Init())
    {
        s_fault_detail = 1U;
        return false;
    }

    if (!LineSensor_IsBackendConfigConfirmed())
    {
        s_fault_detail = 2U;
        (void)LineSensor_Stop();
        return false;
    }

    LineFollow_Init();
    if (!LineFollowControl_Init())
    {
        s_fault_detail = 3U;
        (void)LineSensor_Stop();
        return false;
    }

    s_initialized = true;
    return true;
}

bool Task2LapStop_Start(uint32_t start_timestamp_ms)
{
    if ((!s_initialized) ||
        ((s_state != TASK2_STATE_IDLE) &&
         (s_state != TASK2_STATE_FINISHED)))
    {
        s_fault_detail = 20U;
        (void)BSP_Debug_Printf(
            "T2,START_FAIL=STATE,DETAIL=%lu\r\n",
            (unsigned long)s_fault_detail);
        return false;
    }

    DistanceTracker_Reset();
    LineFollow_Init();
    if (!LineSensor_IsRunning())
    {
        s_fault_detail = 4U;
        (void)BSP_Debug_Printf(
            "T2,START_FAIL=LINE_SENSOR,DETAIL=%lu\r\n",
            (unsigned long)s_fault_detail);
        return false;
    }
    LineSensor_DiscardFrame();

    if (!LineFollowControl_SetBaseSpeedRangeMmps(
            LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S,
            LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S))
    {
        s_fault_detail = 5U;
        (void)BSP_Debug_Printf(
            "T2,START_FAIL=SPEED_RANGE,DETAIL=%lu\r\n",
            (unsigned long)s_fault_detail);
        return false;
    }

    if (!LineFollowControl_Start())
    {
        s_fault_detail = 6U;
        (void)BSP_Debug_Printf(
            "T2,START_FAIL=CONTROL,DETAIL=%lu\r\n",
            (unsigned long)s_fault_detail);
        return false;
    }

    s_state = TASK2_STATE_LAP_RUNNING;
    s_start_ms = start_timestamp_ms;
    s_last_report_ms = start_timestamp_ms;
    s_line_candidate_start_ms = 0U;
    s_line_candidate = false;
    s_line_window_total = 0U;
    s_line_window_matched = 0U;
    s_has_fresh_sensor_frame = false;
    s_last_sensor_sequence = 0U;
    s_last_sensor_valid_mask = 0U;
    s_line_center_mm = 0;
    s_stop_target_center_mm = 0;
    s_stop_offset_mm = 0;
    s_position_speed_mm_s = 0;
    s_fault_detail = 0U;

    (void)BSP_Debug_Printf(
        "T2,START=1,SEARCH_AFTER=%lu,SLOW_AT=%lu,SLOW=%ld,OFFSET=%lu\r\n",
        (unsigned long)TASK2_PREDECEL_DISTANCE_MM,
        (unsigned long)TASK2_PREDECEL_DISTANCE_MM,
        (long)TASK2_PREDECEL_CENTER_SPEED_MM_S,
        (unsigned long)TASK2_SENSOR_FORWARD_OFFSET_MM);
    return true;
}

Task2LapStopResult_t Task2LapStop_Update(uint32_t now_ms)
{
    bool parking_line_black = false;
    bool has_new_line_result = false;
    bool has_new_sensor_frame = false;
    bool distance_stop_completed;

    if ((!s_initialized) || (s_state == TASK2_STATE_FAULT))
    {
        return TASK2_LAP_STOP_RESULT_FAULT;
    }
    if (s_state == TASK2_STATE_FINISHED)
    {
        return TASK2_LAP_STOP_RESULT_FINISHED;
    }
    if (s_state == TASK2_STATE_IDLE)
    {
        return Task2_Fault(6U);
    }

    if (LineSensor_GetFrame(&s_frame_snapshot))
    {
        has_new_sensor_frame =
            (s_frame_snapshot.sequence !=
             s_last_sensor_sequence) ||
            (s_frame_snapshot.valid_mask !=
             s_last_sensor_valid_mask);

        if (has_new_sensor_frame)
        {
            s_last_sensor_sequence =
                s_frame_snapshot.sequence;
            s_last_sensor_valid_mask =
                s_frame_snapshot.valid_mask;

            if (s_frame_snapshot.valid_mask == 0xFFU)
            {
                s_has_fresh_sensor_frame = true;
            }

            /* Ignore invalid startup frames until one fresh valid response. */
            if (s_has_fresh_sensor_frame)
            {
                if (!LineFollow_Update(&s_frame_snapshot) ||
                    !LineFollow_GetResult(
                        &s_line_result_snapshot))
                {
                    if ((s_state !=
                         TASK2_STATE_DISTANCE_STOPPING) &&
                        (s_state !=
                         TASK2_STATE_USER_STOPPING))
                    {
                        return Task2_Fault(7U);
                    }
                }
                else
                {
                    has_new_line_result = true;
                    parking_line_black =
                        ((s_line_result_snapshot.black_mask &
                          TASK2_A_LINE_ADC345_MASK) ==
                         TASK2_A_LINE_ADC345_MASK) ||
                        ((s_line_result_snapshot.black_mask &
                          TASK2_A_LINE_ADC456_MASK) ==
                         TASK2_A_LINE_ADC456_MASK);

                    if (!LineFollowControl_Submit(
                            &s_line_result_snapshot) &&
                        (s_state !=
                         TASK2_STATE_DISTANCE_STOPPING) &&
                        (s_state !=
                         TASK2_STATE_USER_STOPPING))
                    {
                        return Task2_Fault(7U);
                    }
                }
            }
        }
    }

    LineFollowControl_Process();

    if (!DistanceTracker_GetStatus(&s_distance_snapshot) ||
        !LineFollowControl_GetStatus(&s_control_snapshot) ||
        !Chassis_GetStatus(&s_chassis_snapshot))
    {
        return Task2_Fault(8U);
    }

    Task2_Report(
        now_ms,
        &s_distance_snapshot,
        &s_control_snapshot,
        &s_chassis_snapshot);

    if ((s_state == TASK2_STATE_DISTANCE_STOPPING) ||
        (s_state == TASK2_STATE_USER_STOPPING))
    {
        if (((uint32_t)(now_ms - s_start_ms) >=
             TASK2_RUN_TIMEOUT_MS) &&
            !Chassis_IsMotionStopped())
        {
            return Task2_Fault(13U);
        }

        if (!LineFollowControl_IsStopping() &&
            Chassis_IsMotionStopped())
        {
            distance_stop_completed =
                (s_state == TASK2_STATE_DISTANCE_STOPPING);
            s_stop_offset_mm = distance_stop_completed ?
                Task2_AbsI32(
                    s_distance_snapshot.center_signed_mm -
                    s_line_center_mm) :
                0;
            s_state = TASK2_STATE_FINISHED;

            (void)BSP_Debug_Printf(
                "T2,FINISH=1,MS=%lu,DIST=%lu,OFFSET=%ld,ERR=%ld\r\n",
                (unsigned long)(now_ms - s_start_ms),
                (unsigned long)s_distance_snapshot.traveled_mm,
                (long)s_stop_offset_mm,
                distance_stop_completed ?
                    (long)(s_stop_offset_mm -
                           (int32_t)TASK2_SENSOR_FORWARD_OFFSET_MM) :
                    0L);
            LineFollowControl_Shutdown();
            return TASK2_LAP_STOP_RESULT_FINISHED;
        }
        return TASK2_LAP_STOP_RESULT_RUNNING;
    }

    if ((uint32_t)(now_ms - s_start_ms) >= TASK2_RUN_TIMEOUT_MS)
    {
        return Task2_Fault(9U);
    }

    if (!LineFollowControl_IsRunning())
    {
        return Task2_Fault(10U);
    }

    if (s_state == TASK2_STATE_POSITION_APPROACH)
    {
        int32_t progress_mm =
            Task2_GetPositionProgressMm(&s_distance_snapshot);
        int32_t remaining_mm =
            (int32_t)TASK2_SENSOR_FORWARD_OFFSET_MM -
            progress_mm;

        if (remaining_mm <= 0)
        {
            s_stop_offset_mm = progress_mm;
            s_state = TASK2_STATE_DISTANCE_STOPPING;
            (void)BSP_Debug_Printf(
                "T2,POS_REACHED=1,C=%ld,TARGET=%ld,OFFSET=%ld,ERR=%ld\r\n",
                (long)s_distance_snapshot.center_signed_mm,
                (long)s_stop_target_center_mm,
                (long)s_stop_offset_mm,
                (long)(s_stop_offset_mm -
                       (int32_t)TASK2_SENSOR_FORWARD_OFFSET_MM));
            LineFollowControl_Stop(
                LINE_FOLLOW_CONTROL_STOP_TASK_COMPLETE);
            return TASK2_LAP_STOP_RESULT_RUNNING;
        }

        s_position_speed_mm_s =
            Task2_GetPositionSpeedMmps(remaining_mm);
        if (!LineFollowControl_SetBaseSpeedRangeMmps(
                s_position_speed_mm_s,
                s_position_speed_mm_s))
        {
            return Task2_Fault(12U);
        }
    }

    if ((s_state == TASK2_STATE_LAP_RUNNING) &&
        (s_distance_snapshot.traveled_mm >=
         TASK2_PREDECEL_DISTANCE_MM))
    {
        if (!LineFollowControl_SetBaseSpeedRangeMmps(
                TASK2_PREDECEL_CENTER_SPEED_MM_S,
                TASK2_PREDECEL_MIN_SPEED_MM_S))
        {
            return Task2_Fault(11U);
        }
        s_state = TASK2_STATE_PREDECEL;
        (void)BSP_Debug_Printf(
            "T2,PREDECEL=1,SEARCH_LINE=1,DIST=%lu,TARGET=%ld/%ld\r\n",
            (unsigned long)s_distance_snapshot.traveled_mm,
            (long)TASK2_PREDECEL_CENTER_SPEED_MM_S,
            (long)TASK2_PREDECEL_MIN_SPEED_MM_S);
    }

    if ((s_state == TASK2_STATE_PREDECEL) &&
        has_new_line_result)
    {
        if (!s_line_candidate)
        {
            if (parking_line_black)
            {
                s_line_candidate = true;
                s_line_candidate_start_ms = now_ms;
                s_line_window_total = 1U;
                s_line_window_matched = 1U;
                (void)BSP_Debug_Printf(
                    "T2,A_CAND=1,DIST=%lu,MASK=0x%02X,N=%u,WIN=%lu,PCT=%lu\r\n",
                    (unsigned long)s_distance_snapshot.traveled_mm,
                    (unsigned int)s_line_result_snapshot.black_mask,
                    (unsigned int)s_line_result_snapshot.black_count,
                    (unsigned long)TASK2_A_LINE_CONFIRM_MS,
                    (unsigned long)TASK2_A_LINE_MATCH_PERCENT);
            }
        }
        else
        {
            if (s_line_window_total < UINT8_MAX)
            {
                s_line_window_total++;
            }
            if (parking_line_black &&
                (s_line_window_matched < UINT8_MAX))
            {
                s_line_window_matched++;
            }

            if ((uint32_t)(now_ms - s_line_candidate_start_ms) >=
                TASK2_A_LINE_CONFIRM_MS)
            {
                (void)BSP_Debug_Printf(
                    "T2,A_WIN=1,HIT=%u/%u,OK=%u\r\n",
                    (unsigned int)s_line_window_matched,
                    (unsigned int)s_line_window_total,
                    (((uint32_t)s_line_window_matched * 100U) >=
                     ((uint32_t)s_line_window_total *
                      TASK2_A_LINE_MATCH_PERCENT)) ? 1U : 0U);

                if (((uint32_t)s_line_window_matched * 100U) >=
                    ((uint32_t)s_line_window_total *
                     TASK2_A_LINE_MATCH_PERCENT))
                {
                    if (!Task2_StartPositionApproach(
                            &s_distance_snapshot))
                    {
                        return Task2_Fault(12U);
                    }
                }
                else if (parking_line_black)
                {
                    s_line_candidate_start_ms = now_ms;
                    s_line_window_total = 1U;
                    s_line_window_matched = 1U;
                }
                else
                {
                    s_line_candidate = false;
                    s_line_candidate_start_ms = 0U;
                    s_line_window_total = 0U;
                    s_line_window_matched = 0U;
                }
            }
        }
    }

    return TASK2_LAP_STOP_RESULT_RUNNING;
}

bool Task2LapStop_RequestStop(void)
{
    if (!s_initialized)
    {
        return false;
    }

    if (s_state == TASK2_STATE_FINISHED)
    {
        LineFollowControl_Shutdown();
        return true;
    }

    if (s_state == TASK2_STATE_USER_STOPPING)
    {
        return true;
    }

    if ((s_state != TASK2_STATE_LAP_RUNNING) &&
        (s_state != TASK2_STATE_PREDECEL) &&
        (s_state != TASK2_STATE_POSITION_APPROACH) &&
        (s_state != TASK2_STATE_DISTANCE_STOPPING))
    {
        return false;
    }

    if (LineFollowControl_IsRunning() &&
        !LineFollowControl_RequestStop(
            LINE_FOLLOW_CONTROL_STOP_USER))
    {
        return false;
    }

    s_line_center_mm = 0;
    s_state = TASK2_STATE_USER_STOPPING;
    return true;
}

bool Task2LapStop_IsStopped(void)
{
    return (s_state == TASK2_STATE_FINISHED) ||
           (s_state == TASK2_STATE_IDLE) ||
           (s_state == TASK2_STATE_FAULT) ||
           Chassis_IsMotionStopped();
}

void Task2LapStop_ForceSafeStop(void)
{
    if (LineFollowControl_IsInitialized())
    {
        LineFollowControl_Shutdown();
    }
    s_state = TASK2_STATE_IDLE;
}

const char *Task2LapStop_GetPhaseText(void)
{
    switch (s_state)
    {
        case TASK2_STATE_LAP_RUNNING:
            return "T2 LAP RUN";
        case TASK2_STATE_PREDECEL:
            return "T2 FIND A LINE";
        case TASK2_STATE_POSITION_APPROACH:
            return "T2 POS 85MM";
        case TASK2_STATE_DISTANCE_STOPPING:
            return "T2 A STOPPING";
        case TASK2_STATE_USER_STOPPING:
            return "T2 USER STOP";
        case TASK2_STATE_FINISHED:
            return "T2 A STOPPED";
        case TASK2_STATE_FAULT:
            return "T2 FAULT";
        case TASK2_STATE_IDLE:
        default:
            return "T2 READY";
    }
}

uint32_t Task2LapStop_GetFaultDetail(void)
{
    return s_fault_detail;
}
