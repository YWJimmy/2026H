#include "ball_motion_estimator.h"

#include "ball_motion_estimator_config.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static BallMotionState_t s_state;

static int32_t BallEstimator_ClampI64(int64_t value)
{
    if (value > INT32_MAX)
    {
        return INT32_MAX;
    }
    if (value < INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static int32_t BallEstimator_AbsI32(int32_t value)
{
    if (value >= 0)
    {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

int32_t BallMotionEstimator_PixelToPositionUm(int32_t center_x_px)
{
    int64_t delta_px =
        (int64_t)center_x_px - BALL_CALIBRATION_ORIGIN_PX;
    int32_t span_px = (delta_px >= 0) ?
        BALL_CALIBRATION_POS_SPAN_PX :
        BALL_CALIBRATION_NEG_SPAN_PX;

    return BallEstimator_ClampI64(
        (delta_px * BALL_CALIBRATION_SPAN_UM) / span_px);
}

int32_t BallMotionEstimator_PositionUmToPixel(int32_t position_um)
{
    int32_t span_px = (position_um >= 0) ?
        BALL_CALIBRATION_POS_SPAN_PX :
        BALL_CALIBRATION_NEG_SPAN_PX;
    int64_t px = BALL_CALIBRATION_ORIGIN_PX +
        (((int64_t)position_um * span_px) /
         BALL_CALIBRATION_SPAN_UM);

    return BallEstimator_ClampI64(px);
}

bool BallMotionEstimator_Init(uint32_t now_ms)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.initialized = true;
    s_state.timestamp_ms = now_ms;
    return true;
}

void BallMotionEstimator_Reset(uint32_t now_ms)
{
    (void)BallMotionEstimator_Init(now_ms);
}

static void BallEstimator_Predict(
    uint32_t now_ms,
    int32_t model_accel_mm_s2)
{
    uint32_t dt_ms = now_ms - s_state.timestamp_ms;
    int64_t position_delta;
    int64_t velocity_delta;

    if (dt_ms == 0U)
    {
        s_state.model_accel_um_s2 =
            model_accel_mm_s2 * 1000;
        return;
    }
    if (dt_ms > BALL_ESTIMATOR_MAX_PREDICT_DT_MS)
    {
        dt_ms = BALL_ESTIMATOR_MAX_PREDICT_DT_MS;
    }

    s_state.model_accel_um_s2 =
        model_accel_mm_s2 * 1000;
    position_delta =
        ((int64_t)s_state.velocity_um_s * dt_ms) / 1000LL;
    position_delta +=
        ((int64_t)s_state.model_accel_um_s2 * dt_ms * dt_ms) /
        2000000LL;
    velocity_delta =
        ((int64_t)s_state.model_accel_um_s2 * dt_ms) / 1000LL;
    s_state.position_um = BallEstimator_ClampI64(
        (int64_t)s_state.position_um + position_delta);
    s_state.velocity_um_s = BallEstimator_ClampI64(
        (int64_t)s_state.velocity_um_s + velocity_delta);
    s_state.timestamp_ms = now_ms;
}

bool BallMotionEstimator_Update(
    uint32_t now_ms,
    int32_t model_accel_mm_s2,
    bool has_new_frame,
    uint32_t vision_sequence,
    uint32_t frame_timestamp_ms,
    const VisionBallFrame_t *frame)
{
    int32_t measurement_um;
    int32_t residual_um;
    uint32_t frame_dt_ms;
    int64_t correction;

    if (!s_state.initialized)
    {
        return false;
    }

    BallEstimator_Predict(now_ms, model_accel_mm_s2);
    s_state.measurement_accepted = false;
    if (!has_new_frame)
    {
        return true;
    }

    /* During initial tuning every found frame is usable. */
    if ((frame == NULL) || !frame->found)
    {
        s_state.rejected_frames++;
        return true;
    }

    measurement_um = BallMotionEstimator_PixelToPositionUm(
        (int32_t)frame->center_x);
    if (!s_state.valid)
    {
        s_state.valid = true;
        s_state.position_um = measurement_um;
        s_state.velocity_um_s = 0;
    }
    else
    {
        frame_dt_ms = frame_timestamp_ms -
            s_state.measurement_timestamp_ms;
        if (frame_dt_ms < BALL_ESTIMATOR_MIN_FRAME_DT_MS)
        {
            s_state.rejected_frames++;
            return true;
        }

        if (frame_dt_ms > BALL_ESTIMATOR_MAX_FRAME_DT_MS)
        {
            s_state.position_um = measurement_um;
            s_state.velocity_um_s = 0;
        }
        else
        {
            residual_um = measurement_um - s_state.position_um;
            if (BallEstimator_AbsI32(residual_um) >
                BALL_ESTIMATOR_MAX_RESIDUAL_UM)
            {
                s_state.rejected_frames++;
                return true;
            }

            correction =
                ((int64_t)BALL_ESTIMATOR_ALPHA_Q10 * residual_um) >>
                BALL_ESTIMATOR_Q_SHIFT;
            s_state.position_um = BallEstimator_ClampI64(
                (int64_t)s_state.position_um + correction);
            correction =
                (((int64_t)BALL_ESTIMATOR_BETA_Q10 * residual_um * 1000LL) /
                 (int64_t)frame_dt_ms) >> BALL_ESTIMATOR_Q_SHIFT;
            s_state.velocity_um_s = BallEstimator_ClampI64(
                (int64_t)s_state.velocity_um_s + correction);
        }
    }

    s_state.center_x_px = (int32_t)frame->center_x;
    s_state.confidence_milli = frame->score_milli;
    s_state.vision_sequence = vision_sequence;
    s_state.measurement_timestamp_ms = frame_timestamp_ms;
    s_state.measurement_accepted = true;
    s_state.accepted_frames++;
    return true;
}

bool BallMotionEstimator_GetState(BallMotionState_t *state)
{
    if ((!s_state.initialized) || (state == NULL))
    {
        return false;
    }
    *state = s_state;
    return true;
}
