#include "ball_balance_control.h"

#include "ball_balance_control_config.h"
#include "bsp_servo.h"
#include "stm32f4xx_hal.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static bool s_vision_timeout_reported = false;

static uint32_t s_last_vision_sequence = 0U;
static int32_t s_previous_center_x = 0;

static int32_t s_brake_distance_gain =
    BALL_BALANCE_BRAKE_DISTANCE_GAIN;

static uint32_t s_stuck_counter = 0U;
static uint32_t s_stuck_hold_frames = 0U;

static BallBalanceControlStatus_t s_status;

static int32_t BallBalance_AbsI32(int32_t value)
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

static int32_t BallBalance_Q10Multiply(int32_t gain_q10, int32_t value)
{
    int64_t product = (int64_t)gain_q10 * (int64_t)value;

    /* 保持f426版本的Q10舍入行为。 */
    if (product >= 0)
    {
        product +=
            (int64_t)(1L << (BALL_BALANCE_GAIN_Q_SHIFT - 1));
    }
    else
    {
        product -=
            (int64_t)(1L << (BALL_BALANCE_GAIN_Q_SHIFT - 1));
    }

    product >>= BALL_BALANCE_GAIN_Q_SHIFT;

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

static uint16_t BallBalance_ClampPulse(
    int32_t pulse_us,
    bool braking)
{
    int32_t minimum_us;
    int32_t maximum_us;

    if (braking)
    {
        minimum_us =
            (int32_t)BALL_BALANCE_SERVO_BRAKE_MIN_US;
        maximum_us =
            (int32_t)BALL_BALANCE_SERVO_BRAKE_MAX_US;
    }
    else
    {
        minimum_us =
            (int32_t)BALL_BALANCE_SERVO_PUSH_MIN_US;
        maximum_us =
            (int32_t)BALL_BALANCE_SERVO_PUSH_MAX_US;
    }

    if (pulse_us < minimum_us)
    {
        return (uint16_t)minimum_us;
    }

    if (pulse_us > maximum_us)
    {
        return (uint16_t)maximum_us;
    }

    return (uint16_t)pulse_us;
}

static void BallBalance_ResetMotionEstimate(void)
{
    s_previous_center_x = 0;
    s_status.speed = 0;
    s_status.delta_us = 0;
    s_status.braking = false;
    s_status.stuck_boost_us = 0U;
    s_stuck_counter = 0U;
    s_stuck_hold_frames = 0U;
}

static void BallBalance_SetEvent(BallBalanceEvent_t event)
{
    s_status.last_event = event;
    s_status.event_sequence++;
}

static bool BallBalance_SetPulse(uint16_t pulse_us)
{
    if (pulse_us == s_status.servo_pulse_us)
    {
        return true;
    }

    if (!BSP_Servo_SetPulseUs(pulse_us))
    {
        s_status.mode = BALL_BALANCE_MODE_SERVO_ERROR;
        s_status.servo_error_count++;
        BallBalance_SetEvent(
            BALL_BALANCE_EVENT_SERVO_COMMAND_ERROR);
        return false;
    }

    s_status.servo_pulse_us = pulse_us;
    return true;
}

static bool BallBalance_ReturnCenter(
    BallBalanceMode_t mode,
    BallBalanceEvent_t event)
{
    if (!BallBalance_SetPulse(
            BALL_BALANCE_SERVO_CENTER_US))
    {
        /*
         * BallBalance_SetPulse已经记录SERVO_ERROR和
         * SERVO_COMMAND_ERROR。这里只复位运动估计，
         * 不能再用回中模式和回中事件覆盖真实故障。
         */
        BallBalance_ResetMotionEstimate();
        return false;
    }

    BallBalance_ResetMotionEstimate();
    s_status.mode = mode;
    BallBalance_SetEvent(event);
    return true;
}

static int32_t BallBalance_CalculateControlDelta(void)
{
    int32_t error_abs = BallBalance_AbsI32(s_status.error);
    int32_t speed_abs = BallBalance_AbsI32(s_status.speed);
    bool moving_to_target;

    /*
     * 7a35ce0 convention: error = target_x - center_x.
     * Positive error → ball left of target → need to move right.
     * Moving toward = error and speed have the SAME sign.
     */
    moving_to_target =
        ((s_status.error > 0) && (s_status.speed > 0)) ||
        ((s_status.error < 0) && (s_status.speed < 0));

    if (moving_to_target)
    {
        int32_t brake_dist =
            speed_abs * s_brake_distance_gain;

        if (error_abs < brake_dist)
        {
            /*
             * CLOSE: full reverse brake.
             * brake = BASE + |speed| * SPEED_FACTOR.
             * No explicit cap — servo brake limits (1300–2000)
             * provide the physical ceiling, matching 7a35ce0.
             */
            int32_t brake =
                BALL_BALANCE_BRAKE_BASE_US +
                ((speed_abs *
                  BALL_BALANCE_BRAKE_SPEED_NUMERATOR) /
                 BALL_BALANCE_BRAKE_SPEED_DENOMINATOR);

            s_status.braking = true;

            /*
             * 钢球向右运动时向左制动（负delta）；
             * 钢球向左运动时向右制动（正delta）。
             */
            return (s_status.speed > 0) ? -brake : brake;
        }
        else
        {
            /*
             * FAR: keep pushing toward target.
             * 7a35ce0: use KP * error, not half brake.
             */
            s_status.braking = false;
            return BallBalance_Q10Multiply(
                BALL_BALANCE_PUSH_KP_Q10,
                s_status.error);
        }
    }

    /*
     * Ball is moving away or stopped.
     * 7a35ce0: moving away → half brake; stopped → push.
     */
    if (speed_abs > BALL_BALANCE_STUCK_SPEED_PX)
    {
        /* Moving away: half brake to dampen oscillation. */
        int32_t brake =
            (BALL_BALANCE_BRAKE_BASE_US / 2) +
            ((speed_abs *
              BALL_BALANCE_BRAKE_SPEED_NUMERATOR) /
             (BALL_BALANCE_BRAKE_SPEED_DENOMINATOR * 2));

        s_status.braking = true;
        return (s_status.speed > 0) ? -brake : brake;
    }
    else
    {
        /* Stopped: push toward target. */
        s_status.braking = false;
        return BallBalance_Q10Multiply(
            BALL_BALANCE_PUSH_KP_Q10,
            s_status.error);
    }
}

static bool BallBalance_ProcessFoundTarget(uint16_t center_x)
{
    int32_t delta;
    int32_t raw_pulse;
    uint16_t clamped_pulse;

    s_status.center_x = (int32_t)center_x;
    /*
     * 7a35ce0 convention: error = target - center.
     * Positive error → ball is left of target → need to push right.
     */
    s_status.error =
        (int32_t)s_status.target_x - s_status.center_x;
    s_status.has_target = true;
    s_status.lost_frames = 0U;
    s_vision_timeout_reported = false;

    if (s_previous_center_x != 0)
    {
        /* 正值表示钢球向图像右侧移动。 */
        s_status.speed =
            s_status.center_x - s_previous_center_x;
    }
    else
    {
        s_status.speed = 0;
    }
    s_previous_center_x = s_status.center_x;

    if (BallBalance_AbsI32(s_status.error) <=
        BALL_BALANCE_DEADZONE_PX)
    {
        s_status.delta_us = 0;
        s_status.braking = false;
        s_status.mode = BALL_BALANCE_MODE_HOLD;
        return BallBalance_SetPulse(
            BALL_BALANCE_SERVO_CENTER_US);
    }

    delta = BallBalance_CalculateControlDelta();

    /*
     * 7a35ce0 burst-based stuck detection:
     * Wait STUCK_WAIT frames when speed is low and not braking,
     * then apply STUCK_BOOST_US for STUCK_HOLD frames.
     */
    if (s_stuck_hold_frames > 0U)
    {
        /* Currently in boost hold phase: maintain burst. */
        s_stuck_hold_frames--;
        if (delta > 0)
        {
            delta += (int32_t)BALL_BALANCE_STUCK_BOOST_US;
        }
        else
        {
            delta -= (int32_t)BALL_BALANCE_STUCK_BOOST_US;
        }
        s_status.stuck_boost_us =
            BALL_BALANCE_STUCK_BOOST_US;
    }
    else if ((BallBalance_AbsI32(s_status.speed) <
              BALL_BALANCE_STUCK_SPEED_PX) &&
             !s_status.braking)
    {
        s_stuck_counter++;
        if (s_stuck_counter >= BALL_BALANCE_STUCK_WAIT)
        {
            /* Trigger burst. */
            s_stuck_counter = 0U;
            s_stuck_hold_frames =
                BALL_BALANCE_STUCK_HOLD;
            if (delta > 0)
            {
                delta +=
                    (int32_t)BALL_BALANCE_STUCK_BOOST_US;
            }
            else
            {
                delta -=
                    (int32_t)BALL_BALANCE_STUCK_BOOST_US;
            }
            s_status.stuck_boost_us =
                BALL_BALANCE_STUCK_BOOST_US;
        }
        else
        {
            s_status.stuck_boost_us = 0U;
        }
    }
    else
    {
        s_stuck_counter = 0U;
        s_stuck_hold_frames = 0U;
        s_status.stuck_boost_us = 0U;
    }

    if ((delta > 0) &&
        (delta < BALL_BALANCE_MIN_STEP_US))
    {
        delta = BALL_BALANCE_MIN_STEP_US;
    }
    else if ((delta < 0) &&
             (delta > -BALL_BALANCE_MIN_STEP_US))
    {
        delta = -BALL_BALANCE_MIN_STEP_US;
    }

    s_status.delta_us = delta;
    raw_pulse =
        (int32_t)BALL_BALANCE_SERVO_CENTER_US + delta;
    clamped_pulse = BallBalance_ClampPulse(
        raw_pulse,
        s_status.braking ||
        (s_stuck_hold_frames > 0U));

    s_status.mode = s_status.braking
        ? BALL_BALANCE_MODE_BRAKE
        : BALL_BALANCE_MODE_PUSH;

    return BallBalance_SetPulse(clamped_pulse);
}

static bool BallBalance_ProcessNoTarget(void)
{
    s_status.has_target = false;
    s_status.lost_frames++;
    s_status.mode = BALL_BALANCE_MODE_NO_TARGET;
    BallBalance_ResetMotionEstimate();

    if (s_status.lost_frames >=
        BALL_BALANCE_LOST_TIMEOUT_FRAMES)
    {
        s_status.lost_frames = 0U;
        return BallBalance_ReturnCenter(
            BALL_BALANCE_MODE_NO_TARGET,
            BALL_BALANCE_EVENT_LOST_RETURN_CENTER);
    }

    return true;
}

bool BallBalanceControl_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));

    s_initialized = false;
    s_vision_timeout_reported = false;
    s_last_vision_sequence = 0U;
    s_previous_center_x = 0;
    s_brake_distance_gain =
        BALL_BALANCE_BRAKE_DISTANCE_GAIN;

    s_status.mode = BALL_BALANCE_MODE_IDLE;
    s_status.last_event = BALL_BALANCE_EVENT_NONE;
    s_status.target_x = BALL_BALANCE_TARGET_CX_A;
    s_status.servo_pulse_us =
        BALL_BALANCE_SERVO_CENTER_US;
    s_status.timestamp_ms = HAL_GetTick();

    if (!BSP_Servo_Init())
    {
        return false;
    }

    if (!BSP_Servo_SetPulseUs(
            BALL_BALANCE_SERVO_CENTER_US))
    {
        return false;
    }

    if (!BSP_Servo_Enable())
    {
        return false;
    }

    s_initialized = true;
    s_status.initialized = true;
    s_status.servo_enabled = true;
    s_status.mode = BALL_BALANCE_MODE_WAITING_VISION;
    s_status.servo_pulse_us = BSP_Servo_GetPulseUs();
    return true;
}

bool BallBalanceControl_SetTargetX(int32_t target_x)
{
    if (!s_initialized ||
        (target_x < 0) ||
        (target_x > BALL_BALANCE_VISION_X_MAX))
    {
        return false;
    }

    if (target_x == s_status.target_x)
    {
        return true;
    }

    s_status.target_x = target_x;
    s_status.error =
        (int32_t)s_status.target_x - s_status.center_x;
    s_status.stuck_boost_us = 0U;
    s_stuck_counter = 0U;
    s_stuck_hold_frames = 0U;
    return true;
}

bool BallBalanceControl_SetBrakeGain(int32_t gain)
{
    if (gain <= 0)
    {
        return false;
    }

    s_brake_distance_gain = gain;
    return true;
}

int32_t BallBalanceControl_GetBrakeGain(void)
{
    return s_brake_distance_gain;
}

bool BallBalanceControl_Update(
    const VisionStatus_t *vision_status)
{
    bool result = true;

    if (!s_initialized || (vision_status == NULL))
    {
        return false;
    }

    s_status.timestamp_ms = HAL_GetTick();
    s_status.vision_sequence = vision_status->sequence;
    s_status.vision_has_frame = vision_status->has_frame;
    s_status.vision_data_valid = vision_status->data_valid;

    if (vision_status->has_frame &&
        (vision_status->sequence != s_last_vision_sequence))
    {
        s_last_vision_sequence = vision_status->sequence;
        s_status.control_sequence++;

        if (vision_status->frame.found)
        {
            result = BallBalance_ProcessFoundTarget(
                vision_status->frame.center_x);
        }
        else
        {
            result = BallBalance_ProcessNoTarget();
        }
    }

    /*
     * 保留f426行为：最近一帧超过100 ms后回中，
     * 并且同一次超时只产生一次事件。
     */
    if (vision_status->has_frame &&
        !vision_status->data_valid)
    {
        s_status.has_target = false;
        s_status.lost_frames = 0U;

        if (!s_vision_timeout_reported)
        {
            result = BallBalance_ReturnCenter(
                BALL_BALANCE_MODE_VISION_TIMEOUT,
                BALL_BALANCE_EVENT_VISION_TIMEOUT_RETURN_CENTER)
                && result;
            s_vision_timeout_reported = true;
        }
    }

    return result;
}

void BallBalanceControl_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    (void)BSP_Servo_SetPulseUs(
        BALL_BALANCE_SERVO_CENTER_US);
    s_status.servo_pulse_us =
        BALL_BALANCE_SERVO_CENTER_US;

    BSP_Servo_Disable();

    s_initialized = false;
    s_status.initialized = false;
    s_status.servo_enabled = false;
    s_status.has_target = false;
    s_status.mode = BALL_BALANCE_MODE_IDLE;
    BallBalance_ResetMotionEstimate();
}

bool BallBalanceControl_IsInitialized(void)
{
    return s_initialized;
}

bool BallBalanceControl_GetStatus(
    BallBalanceControlStatus_t *status)
{
    if ((status == NULL) || !s_status.initialized)
    {
        return false;
    }

    s_status.brake_distance_gain =
        s_brake_distance_gain;
    *status = s_status;
    return true;
}

const char *BallBalanceControl_ModeName(BallBalanceMode_t mode)
{
    switch (mode)
    {
        case BALL_BALANCE_MODE_IDLE:
            return "IDLE";
        case BALL_BALANCE_MODE_WAITING_VISION:
            return "WAIT";
        case BALL_BALANCE_MODE_NO_TARGET:
            return "NO_TARGET";
        case BALL_BALANCE_MODE_HOLD:
            return "HOLD";
        case BALL_BALANCE_MODE_PUSH:
            return "PUSH";
        case BALL_BALANCE_MODE_BRAKE:
            return "BRAKE";
        case BALL_BALANCE_MODE_VISION_TIMEOUT:
            return "VISION_TO";
        case BALL_BALANCE_MODE_SERVO_ERROR:
            return "SERVO_ERR";
        default:
            return "UNKNOWN";
    }
}

const char *BallBalanceControl_EventName(
    BallBalanceEvent_t event)
{
    switch (event)
    {
        case BALL_BALANCE_EVENT_NONE:
            return "NONE";
        case BALL_BALANCE_EVENT_LOST_RETURN_CENTER:
            return "LOST_RETURN_CENTER";
        case BALL_BALANCE_EVENT_VISION_TIMEOUT_RETURN_CENTER:
            return "VISION_TIMEOUT_RETURN_CENTER";
        case BALL_BALANCE_EVENT_SERVO_COMMAND_ERROR:
            return "SERVO_COMMAND_ERROR";
        default:
            return "UNKNOWN";
    }
}
