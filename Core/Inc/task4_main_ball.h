#ifndef TASK4_MAIN_BALL_H
#define TASK4_MAIN_BALL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "vision.h"

typedef enum
{
    TASK4_MAIN_BALL_MODE_IDLE = 0,
    TASK4_MAIN_BALL_MODE_WAIT_VISION,
    TASK4_MAIN_BALL_MODE_CAPTURE_TARGET,
    TASK4_MAIN_BALL_MODE_HOLD,
    TASK4_MAIN_BALL_MODE_DAMP,
    TASK4_MAIN_BALL_MODE_CORRECT,
    TASK4_MAIN_BALL_MODE_RECOVER,
    TASK4_MAIN_BALL_MODE_VISION_LOST,
    TASK4_MAIN_BALL_MODE_SERVO_ERROR
} Task4MainBallMode_t;

typedef struct
{
    int32_t target_x;
    int32_t target_mm;
    int32_t center_x;
    int32_t position_mm;
    int32_t error_mm;
    int32_t error_px;
    int32_t predicted_error_px;

    /* Compatibility diagnostics: displacement per accepted vision frame. */
    int32_t raw_speed_px;
    int32_t filtered_speed_px;

    /* Time-normalized velocity used by the controller. */
    int32_t raw_velocity_px_s;
    int32_t filtered_velocity_px_s;

    int32_t learned_bias_us;
    int32_t position_control_us;
    int32_t damping_control_us;
    int32_t acceleration_feedforward_us;
    int32_t control_delta_us;

    int32_t chassis_forward_speed_mm_s;
    int32_t chassis_planned_accel_mm_s2;
    int32_t chassis_measured_accel_mm_s2;
    int32_t chassis_turn_command_mm_s;

    uint16_t servo_pulse_us;
    uint16_t vision_dt_ms;
    uint16_t servo_step_limit_us;

    uint32_t max_abs_error_px;
    uint32_t max_abs_error_mm;
    uint32_t vision_sequence;
    uint32_t control_sequence;
    uint32_t lost_frames;

    bool initialized;
    bool target_locked;
    bool capture_target_pending;
    bool vision_valid;
    bool within_one_cm;
    bool one_cm_violation_latched;
    bool dynamic_motion;
    bool launch_boost_active;
    Task4MainBallMode_t mode;
} Task4MainBallStatus_t;

bool Task4MainBall_Init(void);
/* Fixed target used by Task 5 and other hold tasks. */
bool Task4MainBall_InitTarget(int32_t target_x, int32_t target_mm);
/* Capture the current stable ball position before Task 6 starts driving. */
bool Task4MainBall_InitCaptureCurrent(void);
bool Task4MainBall_IsTargetLocked(void);
bool Task4MainBall_Update(const VisionStatus_t *vision_status);
void Task4MainBall_Stop(void);
bool Task4MainBall_IsInitialized(void);
bool Task4MainBall_GetStatus(Task4MainBallStatus_t *status);
const char *Task4MainBall_ModeName(Task4MainBallMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* TASK4_MAIN_BALL_H */
