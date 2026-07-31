#include "line_follow_control.h"

#include "chassis.h"
#include "line_follow_control_config.h"
#include "stm32f4xx_hal.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static bool s_running = false;
static bool s_stopping = false;
static bool s_has_previous_normal_error = false;
static bool s_has_normal_direction = false;

static int16_t s_previous_normal_error = 0;
static int16_t s_error_history[LINE_FOLLOW_CONTROL_ERROR_MEDIAN_WINDOW];
static uint8_t s_error_history_count = 0U;
static uint8_t s_error_history_index = 0U;
static int8_t s_last_search_direction = 0;
static int32_t s_correction_ramped_mm_s = 0;
static int32_t s_center_speed_mm_s =
    LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S;
static int32_t s_min_base_speed_mm_s =
    LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S;

static uint32_t s_state_start_ms = 0U;
static uint32_t s_last_normal_update_ms = 0U;

static LineFollowControlStatus_t s_status;

static int32_t LineFollowControl_AbsI32(int32_t value)
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

static int32_t LineFollowControl_ClampI32(
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

static int32_t LineFollowControl_ApproachI32(
    int32_t current,
    int32_t target,
    int32_t step)
{
    int64_t difference;

    if (step <= 0)
    {
        return target;
    }

    difference = (int64_t)target - (int64_t)current;
    if (difference > (int64_t)step)
    {
        return current + step;
    }

    if (difference < -(int64_t)step)
    {
        return current - step;
    }

    return target;
}

static int32_t LineFollowControl_Q10Multiply(
    int32_t gain_q10,
    int32_t value)
{
    int64_t product = (int64_t)gain_q10 * (int64_t)value;

    if (product >= 0)
    {
        product +=
            (int64_t)(1L << (LINE_FOLLOW_CONTROL_Q_SHIFT - 1));
    }
    else
    {
        product -=
            (int64_t)(1L << (LINE_FOLLOW_CONTROL_Q_SHIFT - 1));
    }

    product >>= LINE_FOLLOW_CONTROL_Q_SHIFT;

    if (product > INT32_MAX)
    {
        return INT32_MAX;
    }

    if (product < INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)product;
}

static int16_t LineFollowControl_Median3(
    int16_t a,
    int16_t b,
    int16_t c)
{
    if (a > b)
    {
        int16_t temporary = a;
        a = b;
        b = temporary;
    }

    if (b > c)
    {
        int16_t temporary = b;
        b = c;
        c = temporary;
    }

    if (a > b)
    {
        b = a;
    }

    return b;
}

static int16_t LineFollowControl_FilterError(int16_t raw_error)
{
    int16_t filtered_error;

    s_error_history[s_error_history_index] = raw_error;
    s_error_history_index = (uint8_t)(
        (s_error_history_index + 1U) %
        LINE_FOLLOW_CONTROL_ERROR_MEDIAN_WINDOW);

    if (s_error_history_count <
        LINE_FOLLOW_CONTROL_ERROR_MEDIAN_WINDOW)
    {
        s_error_history_count++;
    }

    if (s_error_history_count <
        LINE_FOLLOW_CONTROL_ERROR_MEDIAN_WINDOW)
    {
        filtered_error = raw_error;
    }
    else
    {
        filtered_error = LineFollowControl_Median3(
            s_error_history[0],
            s_error_history[1],
            s_error_history[2]);
    }

    if (LineFollowControl_AbsI32((int32_t)filtered_error) <=
        LINE_FOLLOW_CONTROL_ERROR_DEADBAND)
    {
        filtered_error = 0;
    }

    return filtered_error;
}

static int32_t LineFollowControl_GetBaseSpeed(int16_t error)
{
    int32_t error_abs =
        LineFollowControl_AbsI32((int32_t)error);
    int32_t speed_span =
        s_center_speed_mm_s -
        s_min_base_speed_mm_s;
    int32_t reduction;

    error_abs = LineFollowControl_ClampI32(
        error_abs,
        0,
        LINE_FOLLOW_CONTROL_ERROR_FULL_SCALE);

    reduction =
        (int32_t)(((int64_t)speed_span * (int64_t)error_abs) /
                  (int64_t)LINE_FOLLOW_CONTROL_ERROR_FULL_SCALE);

    return s_center_speed_mm_s - reduction;
}

static int32_t LineFollowControl_GetCorrectionStep(uint32_t dt_ms)
{
    int64_t step;

    if (dt_ms == 0U)
    {
        dt_ms = 1U;
    }

    step =
        ((int64_t)LINE_FOLLOW_CONTROL_CORRECTION_SLEW_MM_S2 *
         (int64_t)dt_ms +
         999LL) /
        1000LL;

    if (step < 1LL)
    {
        step = 1LL;
    }
    if (step > INT32_MAX)
    {
        return INT32_MAX;
    }

    return (int32_t)step;
}

static bool LineFollowControl_SetBaseTurnTargets(
    int32_t base_mm_s,
    int32_t turn_mm_s)
{
    int32_t max_turn_mm_s;
    int32_t left_mm_s;
    int32_t right_mm_s;

    base_mm_s = LineFollowControl_ClampI32(
        base_mm_s,
        0,
        LINE_FOLLOW_CONTROL_MAX_WHEEL_SPEED_MM_S);

    max_turn_mm_s =
        LINE_FOLLOW_CONTROL_MAX_WHEEL_SPEED_MM_S - base_mm_s;
    if (base_mm_s < max_turn_mm_s)
    {
        max_turn_mm_s = base_mm_s;
    }
    if (max_turn_mm_s < 0)
    {
        max_turn_mm_s = 0;
    }

    turn_mm_s = LineFollowControl_ClampI32(
        turn_mm_s,
        -max_turn_mm_s,
        max_turn_mm_s);

    left_mm_s = base_mm_s + turn_mm_s;
    right_mm_s = base_mm_s - turn_mm_s;

    s_status.base_speed_mm_s = base_mm_s;
    s_status.correction_mm_s = turn_mm_s;
    s_status.left_target_mm_s = left_mm_s;
    s_status.right_target_mm_s = right_mm_s;

    if (!Chassis_SetLineFollowCommandMmps(
            base_mm_s,
            turn_mm_s))
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
        return false;
    }

    return true;
}

static void LineFollowControl_EnterMode(
    LineFollowControlMode_t mode,
    uint32_t now_ms)
{
    if (s_status.mode != mode)
    {
        s_status.mode = mode;
        s_state_start_ms = now_ms;
    }

    s_status.state_elapsed_ms =
        (uint32_t)(now_ms - s_state_start_ms);
}

static void LineFollowControl_ResetErrorFilter(void)
{
    memset(s_error_history, 0, sizeof(s_error_history));
    s_error_history_count = 0U;
    s_error_history_index = 0U;
}

static void LineFollowControl_ResetPd(void)
{
    s_has_previous_normal_error = false;
    s_previous_normal_error = 0;
    s_correction_ramped_mm_s = 0;
    s_last_normal_update_ms = 0U;
    s_status.error_delta = 0;
    s_status.correction_target_mm_s = 0;
    s_status.correction_mm_s = 0;
}

static void LineFollowControl_ResetStartupGate(void)
{
    s_status.center_confirmed_ms = 0U;
}

static bool LineFollowControl_HandleNormal(
    const LineFollowResult_t *result,
    uint32_t now_ms)
{
    int16_t filtered_error;
    int32_t base_speed;
    int32_t correction_target;
    int32_t p_term;
    int32_t d_term;
    int32_t error_delta;
    int32_t max_correction_by_base;
    int32_t correction_step;
    uint32_t dt_ms;

    filtered_error = LineFollowControl_FilterError(result->error);
    s_status.raw_error = result->error;
    s_status.error = filtered_error;

    if (s_status.mode == LINE_FOLLOW_CONTROL_MODE_WAITING_LINE)
    {
        /* Any valid non-empty line starts control immediately. */
        LineFollowControl_ResetStartupGate();
        LineFollowControl_EnterMode(
            LINE_FOLLOW_CONTROL_MODE_NORMAL,
            now_ms);
    }
    else
    {
        LineFollowControl_EnterMode(
            LINE_FOLLOW_CONTROL_MODE_NORMAL,
            now_ms);
    }

    base_speed = LineFollowControl_GetBaseSpeed(filtered_error);

    if (s_has_previous_normal_error)
    {
        error_delta =
            (int32_t)filtered_error -
            (int32_t)s_previous_normal_error;
    }
    else
    {
        error_delta = 0;
    }

    error_delta = LineFollowControl_ClampI32(
        error_delta,
        -LINE_FOLLOW_CONTROL_ERROR_DELTA_LIMIT,
        LINE_FOLLOW_CONTROL_ERROR_DELTA_LIMIT);

    p_term = LineFollowControl_Q10Multiply(
        LINE_FOLLOW_CONTROL_KP_Q10,
        (int32_t)filtered_error);
    d_term = LineFollowControl_Q10Multiply(
        LINE_FOLLOW_CONTROL_KD_Q10,
        error_delta);

    correction_target = p_term + d_term;
    correction_target = LineFollowControl_ClampI32(
        correction_target,
        -LINE_FOLLOW_CONTROL_MAX_CORRECTION_MM_S,
        LINE_FOLLOW_CONTROL_MAX_CORRECTION_MM_S);

    max_correction_by_base =
        LINE_FOLLOW_CONTROL_MAX_WHEEL_SPEED_MM_S - base_speed;
    if (base_speed < max_correction_by_base)
    {
        max_correction_by_base = base_speed;
    }
    if (max_correction_by_base < 0)
    {
        max_correction_by_base = 0;
    }

    correction_target = LineFollowControl_ClampI32(
        correction_target,
        -max_correction_by_base,
        max_correction_by_base);

    if (s_last_normal_update_ms == 0U)
    {
        dt_ms = 5U;
    }
    else
    {
        dt_ms = (uint32_t)(now_ms - s_last_normal_update_ms);
        if ((dt_ms == 0U) || (dt_ms > 20U))
        {
            dt_ms = 5U;
        }
    }
    s_last_normal_update_ms = now_ms;

    correction_step = LineFollowControl_GetCorrectionStep(dt_ms);
    s_correction_ramped_mm_s = LineFollowControl_ApproachI32(
        s_correction_ramped_mm_s,
        correction_target,
        correction_step);

    s_previous_normal_error = filtered_error;
    s_has_previous_normal_error = true;

    if (filtered_error > 0)
    {
        s_last_search_direction = 1;
        s_has_normal_direction = true;
    }
    else if (filtered_error < 0)
    {
        s_last_search_direction = -1;
        s_has_normal_direction = true;
    }

    s_status.base_speed_mm_s = base_speed;
    s_status.correction_target_mm_s = correction_target;
    s_status.correction_mm_s = s_correction_ramped_mm_s;
    s_status.error_delta =
        (int16_t)LineFollowControl_ClampI32(
            error_delta,
            INT16_MIN,
            INT16_MAX);
    s_status.last_search_direction = s_last_search_direction;
    s_status.has_normal_direction = s_has_normal_direction;

    return LineFollowControl_SetBaseTurnTargets(
        base_speed,
        s_correction_ramped_mm_s);
}

static bool LineFollowControl_HandleLost(uint32_t now_ms)
{
    if (s_status.mode == LINE_FOLLOW_CONTROL_MODE_WAITING_LINE)
    {
        LineFollowControl_ResetStartupGate();
        return LineFollowControl_SetBaseTurnTargets(0, 0);
    }

    LineFollowControl_EnterMode(
        LINE_FOLLOW_CONTROL_MODE_LOST_SEARCH,
        now_ms);
    LineFollowControl_ResetPd();
    LineFollowControl_ResetErrorFilter();

    if (!s_has_normal_direction ||
        (s_last_search_direction == 0))
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_LOST_NO_DIRECTION);
        return false;
    }

    if (s_status.state_elapsed_ms >
        LINE_FOLLOW_CONTROL_LOST_TIMEOUT_MS)
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_LOST_TIMEOUT);
        return false;
    }

    s_status.base_speed_mm_s =
        LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S / 2;
    s_status.correction_target_mm_s =
        (s_last_search_direction > 0) ?
        (LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S / 2) :
        -(LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S / 2);
    s_status.last_search_direction = s_last_search_direction;

    return LineFollowControl_SetBaseTurnTargets(
        LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S / 2,
        s_status.correction_target_mm_s);
}

static bool LineFollowControl_HandleAllBlack(uint32_t now_ms)
{
    if (s_status.mode == LINE_FOLLOW_CONTROL_MODE_WAITING_LINE)
    {
        LineFollowControl_ResetStartupGate();
        return LineFollowControl_SetBaseTurnTargets(0, 0);
    }

    LineFollowControl_EnterMode(
        LINE_FOLLOW_CONTROL_MODE_ALL_BLACK_PASS,
        now_ms);
    LineFollowControl_ResetPd();
    LineFollowControl_ResetErrorFilter();

    if (s_status.state_elapsed_ms >
        LINE_FOLLOW_CONTROL_ALL_BLACK_TIMEOUT_MS)
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_ALL_BLACK_TIMEOUT);
        return false;
    }

    s_status.correction_target_mm_s = 0;
    return LineFollowControl_SetBaseTurnTargets(
        LINE_FOLLOW_CONTROL_ALL_BLACK_SPEED_MM_S,
        0);
}

bool LineFollowControl_Init(void)
{
    s_initialized = false;
    s_running = false;
    s_stopping = false;
    s_has_previous_normal_error = false;
    s_has_normal_direction = false;
    s_previous_normal_error = 0;
    s_last_search_direction = 0;
    s_correction_ramped_mm_s = 0;
    s_center_speed_mm_s =
        LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S;
    s_min_base_speed_mm_s =
        LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S;
    s_state_start_ms = HAL_GetTick();
    s_last_normal_update_ms = 0U;

    LineFollowControl_ResetErrorFilter();
    LineFollowControl_ResetStartupGate();

    memset(&s_status, 0, sizeof(s_status));
    s_status.mode = LINE_FOLLOW_CONTROL_MODE_IDLE;
    s_status.stop_reason = LINE_FOLLOW_CONTROL_STOP_NONE;
    s_status.line_state = LINE_FOLLOW_STATE_INVALID;
    s_status.timestamp_ms = s_state_start_ms;
    s_status.center_speed_limit_mm_s = s_center_speed_mm_s;
    s_status.min_base_speed_mm_s = s_min_base_speed_mm_s;

    if (!Chassis_Init())
    {
        return false;
    }

    s_initialized = true;
    s_status.initialized = true;
    s_status.stopping = false;
    s_status.chassis_enabled = Chassis_IsEnabled();
    s_status.chassis_motion_stopped = Chassis_IsMotionStopped();
    return true;
}

bool LineFollowControl_Start(void)
{
    uint32_t now_ms;

    if (!s_initialized)
    {
        return false;
    }

    if (!Chassis_Enable(true))
    {
        return false;
    }

    now_ms = HAL_GetTick();
    s_running = true;
    s_stopping = false;
    s_has_previous_normal_error = false;
    s_has_normal_direction = false;
    s_previous_normal_error = 0;
    s_last_search_direction = 0;
    s_correction_ramped_mm_s = 0;
    s_last_normal_update_ms = 0U;

    LineFollowControl_ResetErrorFilter();
    LineFollowControl_ResetStartupGate();

    memset(&s_status, 0, sizeof(s_status));
    s_status.initialized = true;
    s_status.running = true;
    s_status.stopping = false;
    s_status.chassis_enabled = true;
    s_status.chassis_motion_stopped = true;
    s_status.mode = LINE_FOLLOW_CONTROL_MODE_WAITING_LINE;
    s_status.stop_reason = LINE_FOLLOW_CONTROL_STOP_NONE;
    s_status.line_state = LINE_FOLLOW_STATE_INVALID;
    s_status.timestamp_ms = now_ms;
    s_status.center_speed_limit_mm_s = s_center_speed_mm_s;
    s_status.min_base_speed_mm_s = s_min_base_speed_mm_s;

    s_state_start_ms = now_ms;
    if (!Chassis_SetLineFollowCommandMmps(0, 0))
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
        return false;
    }

    return true;
}

bool LineFollowControl_SetBaseSpeedRangeMmps(
    int32_t center_speed_mm_s,
    int32_t minimum_speed_mm_s)
{
    if ((!s_initialized) ||
        (minimum_speed_mm_s <= 0) ||
        (center_speed_mm_s < minimum_speed_mm_s) ||
        (center_speed_mm_s >
         LINE_FOLLOW_CONTROL_MAX_WHEEL_SPEED_MM_S))
    {
        return false;
    }

    s_center_speed_mm_s = center_speed_mm_s;
    s_min_base_speed_mm_s = minimum_speed_mm_s;
    s_status.center_speed_limit_mm_s = center_speed_mm_s;
    s_status.min_base_speed_mm_s = minimum_speed_mm_s;
    return true;
}

bool LineFollowControl_RequestStop(
    LineFollowControlStopReason_t reason)
{
    uint32_t now_ms;

    if ((!s_initialized) || (!s_running))
    {
        return false;
    }

    now_ms = HAL_GetTick();
    s_running = false;
    s_stopping = true;
    s_status.running = false;
    s_status.stopping = true;
    s_status.mode = LINE_FOLLOW_CONTROL_MODE_STOPPING;
    s_status.stop_reason = reason;
    s_status.base_speed_mm_s = 0;
    s_status.correction_target_mm_s = 0;
    s_status.correction_mm_s = 0;
    s_status.left_target_mm_s = 0;
    s_status.right_target_mm_s = 0;
    s_status.timestamp_ms = now_ms;
    s_state_start_ms = now_ms;

    LineFollowControl_ResetPd();
    if (!Chassis_RequestStop(CHASSIS_STOP_MODE_FAST))
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
        return false;
    }

    return true;
}

bool LineFollowControl_RequestStopWithDecel(
    LineFollowControlStopReason_t reason,
    int32_t decel_mm_s2,
    int32_t jerk_mm_s3)
{
    uint32_t now_ms;

    if ((!s_initialized) || (!s_running) ||
        (decel_mm_s2 <= 0) || (jerk_mm_s3 <= 0))
    {
        return false;
    }

    now_ms = HAL_GetTick();
    s_running = false;
    s_stopping = true;
    s_status.running = false;
    s_status.stopping = true;
    s_status.mode = LINE_FOLLOW_CONTROL_MODE_STOPPING;
    s_status.stop_reason = reason;
    s_status.base_speed_mm_s = 0;
    s_status.correction_target_mm_s = 0;
    s_status.correction_mm_s = 0;
    s_status.left_target_mm_s = 0;
    s_status.right_target_mm_s = 0;
    s_status.timestamp_ms = now_ms;
    s_state_start_ms = now_ms;

    LineFollowControl_ResetPd();
    if (!Chassis_RequestStopWithDecel(
            decel_mm_s2,
            jerk_mm_s3))
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR);
        return false;
    }

    return true;
}

void LineFollowControl_Stop(LineFollowControlStopReason_t reason)
{
    if (!s_initialized)
    {
        return;
    }

    s_running = false;
    s_stopping = false;
    s_status.running = false;
    s_status.stopping = false;
    s_status.mode = LINE_FOLLOW_CONTROL_MODE_STOPPED;
    s_status.stop_reason = reason;
    s_status.base_speed_mm_s = 0;
    s_status.correction_target_mm_s = 0;
    s_status.correction_mm_s = 0;
    s_status.left_target_mm_s = 0;
    s_status.right_target_mm_s = 0;
    s_status.timestamp_ms = HAL_GetTick();

    LineFollowControl_ResetPd();
    LineFollowControl_ResetErrorFilter();
    LineFollowControl_ResetStartupGate();

    /* 故障或二次按键急停绕过规划器，立即短路刹车。 */
    Chassis_Stop();
    s_status.chassis_enabled = Chassis_IsEnabled();
    s_status.chassis_motion_stopped = true;
}

void LineFollowControl_Shutdown(void)
{
    if (!s_initialized)
    {
        return;
    }

    LineFollowControl_Stop(LINE_FOLLOW_CONTROL_STOP_USER);
    (void)Chassis_Enable(false);
    s_status.chassis_enabled = false;
    s_status.mode = LINE_FOLLOW_CONTROL_MODE_IDLE;
}

bool LineFollowControl_Submit(const LineFollowResult_t *result)
{
    uint32_t now_ms;

    if ((!s_initialized) || (result == NULL))
    {
        return false;
    }

    now_ms = HAL_GetTick();
    s_status.line_state = result->state;
    s_status.black_mask = result->black_mask;
    s_status.raw_error = result->error;
    s_status.line_sequence = result->sequence;
    s_status.timestamp_ms = now_ms;

    if (!s_running)
    {
        return true;
    }

    switch (result->state)
    {
        case LINE_FOLLOW_STATE_NORMAL:
            return LineFollowControl_HandleNormal(result, now_ms);

        case LINE_FOLLOW_STATE_LOST:
            return LineFollowControl_HandleLost(now_ms);

        case LINE_FOLLOW_STATE_ALL_BLACK:
            return LineFollowControl_HandleAllBlack(now_ms);

        case LINE_FOLLOW_STATE_INVALID:
        default:
            LineFollowControl_Stop(
                LINE_FOLLOW_CONTROL_STOP_INVALID_DATA);
            return false;
    }
}

void LineFollowControl_Process(void)
{
    uint32_t now_ms;

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if (Chassis_IsEnabled())
    {
        (void)Chassis_Update();
    }

    s_status.chassis_enabled = Chassis_IsEnabled();
    s_status.chassis_motion_stopped = Chassis_IsMotionStopped();

    if ((s_status.mode == LINE_FOLLOW_CONTROL_MODE_WAITING_LINE) ||
        (s_status.mode == LINE_FOLLOW_CONTROL_MODE_LOST_SEARCH) ||
        (s_status.mode == LINE_FOLLOW_CONTROL_MODE_ALL_BLACK_PASS) ||
        (s_status.mode == LINE_FOLLOW_CONTROL_MODE_STOPPING))
    {
        s_status.state_elapsed_ms =
            (uint32_t)(now_ms - s_state_start_ms);
    }

    if (s_stopping && Chassis_IsMotionStopped())
    {
        s_stopping = false;
        s_status.stopping = false;
        s_status.mode = LINE_FOLLOW_CONTROL_MODE_STOPPED;
        s_status.timestamp_ms = now_ms;
    }
}

bool LineFollowControl_IsInitialized(void)
{
    return s_initialized;
}

bool LineFollowControl_IsRunning(void)
{
    return s_initialized && s_running;
}

bool LineFollowControl_IsStopping(void)
{
    return s_initialized && s_stopping;
}

bool LineFollowControl_GetStatus(LineFollowControlStatus_t *status)
{
    if ((!s_initialized) || (status == NULL))
    {
        return false;
    }

    *status = s_status;
    return true;
}

const char *LineFollowControl_ModeName(LineFollowControlMode_t mode)
{
    switch (mode)
    {
        case LINE_FOLLOW_CONTROL_MODE_IDLE:
            return "IDLE";
        case LINE_FOLLOW_CONTROL_MODE_WAITING_LINE:
            return "WAIT";
        case LINE_FOLLOW_CONTROL_MODE_NORMAL:
            return "NORMAL";
        case LINE_FOLLOW_CONTROL_MODE_LOST_SEARCH:
            return "LOST";
        case LINE_FOLLOW_CONTROL_MODE_ALL_BLACK_PASS:
            return "BLACK";
        case LINE_FOLLOW_CONTROL_MODE_STOPPING:
            return "STOPPING";
        case LINE_FOLLOW_CONTROL_MODE_STOPPED:
            return "STOP";
        default:
            return "UNKNOWN";
    }
}

const char *LineFollowControl_StopReasonName(
    LineFollowControlStopReason_t reason)
{
    switch (reason)
    {
        case LINE_FOLLOW_CONTROL_STOP_NONE:
            return "NONE";
        case LINE_FOLLOW_CONTROL_STOP_USER:
            return "USER";
        case LINE_FOLLOW_CONTROL_STOP_INVALID_DATA:
            return "INVALID";
        case LINE_FOLLOW_CONTROL_STOP_LOST_TIMEOUT:
            return "LOST_TO";
        case LINE_FOLLOW_CONTROL_STOP_LOST_NO_DIRECTION:
            return "LOST_DIR";
        case LINE_FOLLOW_CONTROL_STOP_ALL_BLACK_TIMEOUT:
            return "BLACK_TO";
        case LINE_FOLLOW_CONTROL_STOP_TASK_COMPLETE:
            return "TASK_DONE";
        case LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR:
            return "CMD_ERR";
        default:
            return "UNKNOWN";
    }
}
