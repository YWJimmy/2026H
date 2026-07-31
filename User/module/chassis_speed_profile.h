#ifndef CHASSIS_SPEED_PROFILE_H
#define CHASSIS_SPEED_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CHASSIS_SPEED_PROFILE_MODE_IDLE = 0,
    CHASSIS_SPEED_PROFILE_MODE_DRIVE,
    CHASSIS_SPEED_PROFILE_MODE_SOFT_STOP,
    CHASSIS_SPEED_PROFILE_MODE_FAST_STOP,
    CHASSIS_SPEED_PROFILE_MODE_REVERSAL_STOP
} ChassisSpeedProfileMode_t;

typedef struct
{
    int32_t max_wheel_speed_mm_s;

    /* 同向速度变化使用较慢的前进参数。 */
    int32_t forward_accel_mm_s2;
    int32_t forward_decel_mm_s2;
    int32_t forward_accel_jerk_mm_s3;
    int32_t forward_decel_jerk_mm_s3;

    /* 左右轮反向变化时使用较快的转向参数。 */
    int32_t turn_accel_mm_s2;
    int32_t turn_jerk_mm_s3;

    int32_t soft_stop_decel_mm_s2;
    int32_t soft_stop_jerk_mm_s3;
    int32_t fast_stop_decel_mm_s2;
    int32_t fast_stop_jerk_mm_s3;

    int32_t speed_snap_mm_s;
    int32_t accel_snap_mm_s2;
} ChassisSpeedProfileConfig_t;

typedef struct
{
    int32_t speed_q16;
    int32_t acceleration_q16;
} ChassisSpeedProfileAxis_t;

typedef struct
{
    ChassisSpeedProfileConfig_t config;

    /* DRIVE模式直接规划左右轮，避免差速命令出现临时反转。 */
    ChassisSpeedProfileAxis_t left_axis;
    ChassisSpeedProfileAxis_t right_axis;

    /* 停车模式使用公共包络，同比例缩放两轮。 */
    ChassisSpeedProfileAxis_t stop_axis;

    int32_t command_left_q16;
    int32_t command_right_q16;
    int32_t output_left_q16;
    int32_t output_right_q16;

    int32_t stop_start_left_q16;
    int32_t stop_start_right_q16;
    int32_t stop_start_reference_q16;

    int32_t pending_left_q16;
    int32_t pending_right_q16;

    ChassisSpeedProfileMode_t mode;
    bool active;
    bool initialized;
} ChassisSpeedProfile_t;

typedef struct
{
    ChassisSpeedProfileMode_t mode;
    bool active;
    bool stopped;

    int32_t command_left_mm_s;
    int32_t command_right_mm_s;
    int32_t output_left_mm_s;
    int32_t output_right_mm_s;

    /* 由左右轮规划状态换算，仅用于诊断。 */
    int32_t forward_speed_mm_s;
    int32_t turn_speed_mm_s;
    int32_t forward_accel_mm_s2;
    int32_t turn_accel_mm_s2;
    int32_t stop_reference_mm_s;
    int32_t stop_accel_mm_s2;
} ChassisSpeedProfileStatus_t;

bool ChassisSpeedProfile_Init(
    ChassisSpeedProfile_t *profile,
    const ChassisSpeedProfileConfig_t *config);

void ChassisSpeedProfile_Reset(ChassisSpeedProfile_t *profile);

bool ChassisSpeedProfile_SetCommandMmps(
    ChassisSpeedProfile_t *profile,
    int32_t left_mm_s,
    int32_t right_mm_s);

bool ChassisSpeedProfile_RequestStop(
    ChassisSpeedProfile_t *profile,
    ChassisSpeedProfileMode_t stop_mode);

/* 将规划器状态同步到外部已生成的实际左右轮目标。 */
bool ChassisSpeedProfile_SynchronizeOutputMmps(
    ChassisSpeedProfile_t *profile,
    int32_t left_mm_s,
    int32_t right_mm_s);

bool ChassisSpeedProfile_Update(
    ChassisSpeedProfile_t *profile,
    uint16_t dt_ms);

bool ChassisSpeedProfile_GetStatus(
    const ChassisSpeedProfile_t *profile,
    ChassisSpeedProfileStatus_t *status);

const char *ChassisSpeedProfile_ModeName(
    ChassisSpeedProfileMode_t mode);

#endif /* CHASSIS_SPEED_PROFILE_H */
