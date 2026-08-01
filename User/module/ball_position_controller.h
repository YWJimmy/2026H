#ifndef BALL_POSITION_CONTROLLER_H
#define BALL_POSITION_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ball_dynamics_model.h"
#include "ball_motion_estimator.h"

typedef enum
{
    BALL_POSITION_CONTROLLER_IDLE = 0,
    BALL_POSITION_CONTROLLER_WAITING_STATE,
    BALL_POSITION_CONTROLLER_MOVING,
    BALL_POSITION_CONTROLLER_HOLDING,
    BALL_POSITION_CONTROLLER_SERVO_FAULT
} BallPositionControllerMode_t;

typedef struct
{
    bool initialized;
    bool enabled;
    bool at_rest;
    BallPositionControllerMode_t mode;
    uint32_t timestamp_ms;
    uint32_t control_sequence;
    uint32_t servo_error_count;
    uint32_t stuck_duration_ms;
    int32_t target_position_um;
    int32_t position_error_um;
    int32_t desired_ball_accel_mm_s2;
    int32_t stuck_compensation_mm_s2;
    int32_t desired_platform_angle_mrad;
    int32_t predicted_ball_accel_mm_s2;
    int32_t chassis_axis_accel_mm_s2;
    uint16_t servo_pulse_us;
} BallPositionControllerStatus_t;

bool BallPositionController_Init(uint32_t now_ms);
bool BallPositionController_SetTarget(
    int32_t target_position_um,
    int32_t tolerance_um,
    int32_t velocity_tolerance_um_s);
bool BallPositionController_Update(
    uint32_t now_ms,
    const BallMotionState_t *motion,
    const BallBaseMotion_t *base_motion);
void BallPositionController_Stop(void);
bool BallPositionController_GetStatus(
    BallPositionControllerStatus_t *status);
int32_t BallPositionController_GetPredictedAccelMmps2(void);
const char *BallPositionController_ModeName(
    BallPositionControllerMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* BALL_POSITION_CONTROLLER_H */
