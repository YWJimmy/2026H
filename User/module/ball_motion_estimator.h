#ifndef BALL_MOTION_ESTIMATOR_H
#define BALL_MOTION_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "vision_protocol.h"

typedef struct
{
    bool initialized;
    bool valid;
    bool measurement_accepted;
    uint32_t timestamp_ms;
    uint32_t measurement_timestamp_ms;
    uint32_t vision_sequence;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    int32_t center_x_px;
    int32_t position_um;
    int32_t measured_velocity_um_s;
    int32_t velocity_um_s;
    int32_t model_accel_um_s2;
    uint16_t confidence_milli;
} BallMotionState_t;

bool BallMotionEstimator_Init(uint32_t now_ms);
bool BallMotionEstimator_Update(
    uint32_t now_ms,
    int32_t model_accel_mm_s2,
    bool has_new_frame,
    uint32_t vision_sequence,
    uint32_t frame_timestamp_ms,
    const VisionBallFrame_t *frame);
void BallMotionEstimator_Reset(uint32_t now_ms);
bool BallMotionEstimator_GetState(BallMotionState_t *state);
int32_t BallMotionEstimator_PixelToPositionUm(int32_t center_x_px);
int32_t BallMotionEstimator_PositionUmToPixel(int32_t position_um);

#ifdef __cplusplus
}
#endif

#endif /* BALL_MOTION_ESTIMATOR_H */
