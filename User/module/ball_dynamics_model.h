#ifndef BALL_DYNAMICS_MODEL_H
#define BALL_DYNAMICS_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool valid;
    uint32_t timestamp_ms;
    int32_t forward_velocity_mm_s;
    int32_t forward_accel_mm_s2;
    int32_t yaw_rate_mrad_s;
    int32_t yaw_accel_mrad_s2;
    int32_t lateral_accel_mm_s2;
    int32_t axis_accel_mm_s2;
} BallBaseMotion_t;

typedef struct
{
    int32_t platform_angle_mrad;
    int32_t chassis_axis_accel_mm_s2;
    int32_t predicted_ball_accel_mm_s2;
    uint16_t servo_pulse_us;
} BallDynamicsStatus_t;

void BallDynamicsModel_ResetBaseMotion(uint32_t now_ms);
bool BallDynamicsModel_GetBaseMotion(
    uint32_t now_ms,
    BallBaseMotion_t *motion);
int32_t BallDynamicsModel_DesiredAccelToAngleMrad(
    int32_t desired_ball_accel_mm_s2,
    int32_t chassis_axis_accel_mm_s2);
int32_t BallDynamicsModel_PredictBallAccelMmps2(
    int32_t platform_angle_mrad,
    int32_t chassis_axis_accel_mm_s2);
uint16_t BallDynamicsModel_AngleToPulseUs(int32_t angle_mrad);
int32_t BallDynamicsModel_PulseToAngleMrad(uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* BALL_DYNAMICS_MODEL_H */
