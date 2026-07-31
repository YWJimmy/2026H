#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

/* 用户实测并确认的底盘机械参数。 */
#define CHASSIS_WHEEL_DIAMETER_MM              ((int32_t)65)
#define CHASSIS_TRACK_WIDTH_MM                 ((int32_t)210)

/*
 * 圆周长 = π × 65 mm ≈ 204.2035 mm。
 * 使用微米整数保存，避免运行时浮点运算。
 */
#define CHASSIS_WHEEL_CIRCUMFERENCE_UM         ((int32_t)204204)

/* 5 ms，200 Hz控制周期。 */
#define CHASSIS_CONTROL_PERIOD_MS              ((uint16_t)5U)

/*
 * 主循环若超过该时间没有执行控制更新，则视为严重超时：
 * 丢弃该段编码器增量、清除目标与PI状态并短路刹车。
 */
#define CHASSIS_TIMING_OVERRUN_MS              ((uint16_t)25U)

/* 初期安全限制，低于TIM9硬件最大值8399。 */
#define CHASSIS_PWM_LIMIT                      ((int16_t)5000)

/*
 * 底盘全局轮速上限。
 * 巡线外环也会再次限制为0～500 mm/s，禁止巡线时反转。
 */
#define CHASSIS_MAX_WHEEL_SPEED_MM_S           ((int32_t)500)
#define CHASSIS_MAX_WHEEL_SPEED_CPS            ((int32_t)3594)

/*
 * 最终目标先分解为前进分量与转向分量，再分别限制变化率。
 * 编码器前馈+PI只跟踪斜坡产生的中间目标。
 */
#define CHASSIS_RAMP_FORWARD_ACCEL_MM_S2       ((int32_t)800)
#define CHASSIS_RAMP_FORWARD_DECEL_MM_S2       ((int32_t)1200)
#define CHASSIS_RAMP_TURN_SLEW_MM_S2           ((int32_t)2000)

/*
 * 编码器单周期合理性保护。
 *
 * 正常架空最高速度约5300 count/s。这里放宽到8000 count/s，
 * 并额外允许4个计数，避免量化和调度抖动误判。
 *
 * 5 ms周期下允许：
 * ceil(8000 × 5 / 1000) + 4 = 44 count。
 */
#define CHASSIS_ENCODER_PLAUSIBLE_MAX_CPS      ((int32_t)8000)
#define CHASSIS_ENCODER_DELTA_MARGIN_COUNTS    ((int32_t)4)

/*
 * PI参数采用Q10定点：
 * 实际Kp = KP_Q10 / 1024
 * 实际Ki = KI_Q10 / 1024
 */
#define WHEEL_SPEED_LEFT_KP_Q10                ((int32_t)512)
#define WHEEL_SPEED_LEFT_KI_Q10                ((int32_t)8)
#define WHEEL_SPEED_RIGHT_KP_Q10               ((int32_t)512)
#define WHEEL_SPEED_RIGHT_KI_Q10               ((int32_t)8)

/* 积分项最大修正量。 */
#define WHEEL_SPEED_INTEGRAL_LIMIT_PWM         ((int16_t)2500)

/*
 * 开环前馈：
 * PWM_ff = sign(target) ×
 *          (static_pwm + gain_q10 × abs(target_cps) / 1024)
 */
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

/*
 * 控制每5 ms执行一次，但测速使用最近4个控制周期的滑动窗口：
 * 等效测速窗口约20 ms。
 */
#define WHEEL_SPEED_MEASUREMENT_WINDOW         ((uint8_t)4U)

/* 固定最小PWM补偿保持关闭，由前馈静态项跨越死区。 */
#define WHEEL_SPEED_LEFT_MIN_DRIVE_PWM         ((int16_t)0)
#define WHEEL_SPEED_RIGHT_MIN_DRIVE_PWM        ((int16_t)0)

#endif /* CHASSIS_CONFIG_H */
