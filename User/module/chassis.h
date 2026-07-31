#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CHASSIS_STOP_MODE_SOFT = 0,
    CHASSIS_STOP_MODE_FAST,
    CHASSIS_STOP_MODE_EMERGENCY
} ChassisStopMode_t;

typedef enum
{
    CHASSIS_MOTION_MODE_IDLE = 0,
    CHASSIS_MOTION_MODE_DRIVE,
    CHASSIS_MOTION_MODE_SOFT_STOP,
    CHASSIS_MOTION_MODE_FAST_STOP,
    CHASSIS_MOTION_MODE_REVERSAL_STOP,
    CHASSIS_MOTION_MODE_EMERGENCY
} ChassisMotionMode_t;

typedef struct
{
    bool initialized;
    bool enabled;
    bool encoder_sample_valid;
    bool left_output_saturated;
    bool right_output_saturated;
    bool speed_ramp_active;
    bool motion_stopped;
    bool emergency_latched;
    bool line_follow_active;

    ChassisMotionMode_t motion_mode;

    /* 上层最终左右轮命令。 */
    int32_t left_command_mm_s;
    int32_t right_command_mm_s;
    int32_t left_command_cps;
    int32_t right_command_cps;

    /* 前馈+PI跟踪的规划中间目标。 */
    int32_t left_target_mm_s;
    int32_t right_target_mm_s;
    int32_t left_target_cps;
    int32_t right_target_cps;

    /* 规划器诊断。 */
    int32_t forward_command_mm_s;
    int32_t turn_command_mm_s;
    int32_t forward_target_mm_s;
    int32_t turn_target_mm_s;
    int32_t forward_accel_mm_s2;
    int32_t turn_accel_mm_s2;
    int32_t stop_reference_mm_s;
    int32_t stop_accel_mm_s2;

    /* 巡线专用快转向通道诊断。 */
    int32_t line_follow_turn_command_mm_s;
    int32_t line_follow_turn_ramped_mm_s;

    int32_t left_measured_mm_s;
    int32_t right_measured_mm_s;
    int32_t left_raw_measured_cps;
    int32_t right_raw_measured_cps;
    int32_t left_measured_cps;
    int32_t right_measured_cps;
    int32_t left_error_cps;
    int32_t right_error_cps;

    int16_t left_feedforward_pwm;
    int16_t right_feedforward_pwm;
    int32_t left_proportional_pwm;
    int32_t right_proportional_pwm;
    int32_t left_integral_pwm;
    int32_t right_integral_pwm;
    int16_t left_pwm;
    int16_t right_pwm;

    int16_t left_delta;
    int16_t right_delta;
    int32_t left_total;
    int32_t right_total;

    uint16_t dt_ms;
    uint16_t stopped_stable_ms;
    uint32_t control_sequence;
    uint32_t timestamp_ms;
    uint32_t timing_overrun_count;
    uint32_t left_encoder_reject_count;
    uint32_t right_encoder_reject_count;
} ChassisStatus_t;

bool Chassis_Init(void);
bool Chassis_Enable(bool enable);
bool Chassis_IsInitialized(void);
bool Chassis_IsEnabled(void);

int32_t Chassis_MmpsToCps(int32_t speed_mm_s);
int32_t Chassis_CpsToMmps(int32_t speed_cps);

/* 普通非零命令走限跃度S曲线；0/0自动请求柔和停车。 */
bool Chassis_SetWheelSpeedMmps(int32_t left_mm_s,
                               int32_t right_mm_s);
bool Chassis_SetWheelSpeedCps(int32_t left_cps,
                              int32_t right_cps);

/* 巡线专用：base走慢速S曲线，turn走快速限速率。 */
bool Chassis_SetLineFollowCommandMmps(int32_t base_mm_s,
                                      int32_t turn_mm_s);

bool Chassis_SetVelocity(int32_t linear_mm_s,
                         int32_t angular_mrad_s);

/* 柔和停车、快速停车或立即锁存急停。 */
bool Chassis_RequestStop(ChassisStopMode_t mode);
bool Chassis_RequestStopWithDecel(
    int32_t decel_mm_s2,
    int32_t jerk_mm_s3);
bool Chassis_IsMotionStopped(void);

bool Chassis_SetWheelPiGainsQ10(int32_t left_kp_q10,
                                int32_t left_ki_q10,
                                int32_t right_kp_q10,
                                int32_t right_ki_q10);

bool Chassis_SetWheelFeedforwardQ10(
    int32_t left_gain_q10,
    int16_t left_static_pwm,
    int32_t right_gain_q10,
    int16_t right_static_pwm);

/* 兼容旧接口：立即锁存急停并短路刹车。 */
void Chassis_Stop(void);

bool Chassis_Update(void);
bool Chassis_GetStatus(ChassisStatus_t *status);

const char *Chassis_MotionModeName(ChassisMotionMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
