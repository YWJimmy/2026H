#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

/* 用户实测并确认的底盘机械参数。 */
#define CHASSIS_WHEEL_DIAMETER_MM              ((int32_t)65)
#define CHASSIS_TRACK_WIDTH_MM                 ((int32_t)210)

/* 圆周长 = pi * 65 mm，使用微米整数避免运行时浮点。 */
#define CHASSIS_WHEEL_CIRCUMFERENCE_UM         ((int32_t)204204)

/* 5 ms，200 Hz控制周期。 */
#define CHASSIS_CONTROL_PERIOD_MS              ((uint16_t)5U)

/* 主循环严重超时后立即进入锁存急停。 */
#define CHASSIS_TIMING_OVERRUN_MS              ((uint16_t)25U)

/* 低于TIM9硬件最大值8399。 */
#define CHASSIS_PWM_LIMIT                      ((int16_t)5000)

/* 全局轮速上限。 */
#define CHASSIS_MAX_WHEEL_SPEED_MM_S           ((int32_t)500)
#define CHASSIS_MAX_WHEEL_SPEED_CPS            ((int32_t)3594)

/*
 * V4在线限跃度速度规划参数。
 *
 * 正常前进速度变化较慢，减小滚球平台的纵向冲击；
 * 转向轴保持较高响应，避免巡线方向修正过慢。
 */
#define CHASSIS_PROFILE_FORWARD_ACCEL_MM_S2       ((int32_t)220)
#define CHASSIS_PROFILE_FORWARD_DECEL_MM_S2       ((int32_t)350)
#define CHASSIS_PROFILE_FORWARD_ACCEL_JERK_MM_S3  ((int32_t)1100)
#define CHASSIS_PROFILE_FORWARD_DECEL_JERK_MM_S3  ((int32_t)1800)

#define CHASSIS_PROFILE_TURN_ACCEL_MM_S2          ((int32_t)1000)
#define CHASSIS_PROFILE_TURN_JERK_MM_S3           ((int32_t)5000)

/*
 * 巡线专用双通道：基础速度继续走慢速S曲线，转向修正走快速限速率。
 * 该通道只允许非负轮速，并使用独立的巡线轮速上限。
 */
#define CHASSIS_LINE_FOLLOW_MAX_WHEEL_SPEED_MM_S  ((int32_t)500)
#define CHASSIS_LINE_FOLLOW_TURN_SLEW_MM_S2        ((int32_t)6000)

/* 普通命令0/0使用同比例柔和停车。 */
#define CHASSIS_PROFILE_SOFT_STOP_DECEL_MM_S2     ((int32_t)600)
#define CHASSIS_PROFILE_SOFT_STOP_JERK_MM_S3      ((int32_t)3000)

/* KEY0普通停止使用更快但仍限跃度的停车。 */
#define CHASSIS_PROFILE_FAST_STOP_DECEL_MM_S2     ((int32_t)900)
#define CHASSIS_PROFILE_FAST_STOP_JERK_MM_S3      ((int32_t)6000)

/* 目标吸附容差，修复整数分解后A长期为1。 */
#define CHASSIS_PROFILE_SPEED_SNAP_MM_S            ((int32_t)2)
#define CHASSIS_PROFILE_ACCEL_SNAP_MM_S2            ((int32_t)20)

/* RMP归零后，编码器速度连续满足条件才报告完全停车。 */
#define CHASSIS_STOP_MEASURED_THRESHOLD_MM_S        ((int32_t)15)
#define CHASSIS_STOP_STABLE_HOLD_MS                  ((uint16_t)50U)

/* 编码器单周期合理性保护。 */
#define CHASSIS_ENCODER_PLAUSIBLE_MAX_CPS      ((int32_t)8000)
#define CHASSIS_ENCODER_DELTA_MARGIN_COUNTS    ((int32_t)4)

/* PI参数采用Q10定点。 */
#define WHEEL_SPEED_LEFT_KP_Q10                ((int32_t)512)
#define WHEEL_SPEED_LEFT_KI_Q10                ((int32_t)8)
#define WHEEL_SPEED_RIGHT_KP_Q10               ((int32_t)512)
#define WHEEL_SPEED_RIGHT_KI_Q10               ((int32_t)8)

#define WHEEL_SPEED_INTEGRAL_LIMIT_PWM         ((int16_t)2500)

#define WHEEL_SPEED_FEEDFORWARD_ENABLE         1U

#if WHEEL_SPEED_FEEDFORWARD_ENABLE
#define WHEEL_SPEED_LEFT_FF_GAIN_Q10           ((int32_t)937)
#define WHEEL_SPEED_LEFT_FF_STATIC_PWM         ((int16_t)124)
#define WHEEL_SPEED_RIGHT_FF_GAIN_Q10          ((int32_t)954)
#define WHEEL_SPEED_RIGHT_FF_STATIC_PWM        ((int16_t)183)
#else
#define WHEEL_SPEED_LEFT_FF_GAIN_Q10           ((int32_t)0)
#define WHEEL_SPEED_LEFT_FF_STATIC_PWM         ((int16_t)0)
#define WHEEL_SPEED_RIGHT_FF_GAIN_Q10          ((int32_t)0)
#define WHEEL_SPEED_RIGHT_FF_STATIC_PWM        ((int16_t)0)
#endif

/* 约20 ms测速窗口。 */
#define WHEEL_SPEED_MEASUREMENT_WINDOW         ((uint8_t)4U)

#define WHEEL_SPEED_LEFT_MIN_DRIVE_PWM         ((int16_t)0)
#define WHEEL_SPEED_RIGHT_MIN_DRIVE_PWM        ((int16_t)0)

#endif /* CHASSIS_CONFIG_H */
