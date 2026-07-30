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
 * 丢弃该段编码器增量、清除PI状态并短路刹车。
 */
#define CHASSIS_TIMING_OVERRUN_MS              ((uint16_t)25U)

/* 初期安全限制，低于TIM9硬件最大值8399。 */
#define CHASSIS_PWM_LIMIT                      ((int16_t)5000)

/*
 * 根据共地修复后的架空开环测试：
 * PWM=5000时左右轮约为5305/5150 count/s，
 * 对应约738/717 mm/s。
 */
#define CHASSIS_MAX_WHEEL_SPEED_MM_S           ((int32_t)700)
#define CHASSIS_MAX_WHEEL_SPEED_CPS            ((int32_t)5032)

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
 *
 * V2先保留已经实测稳定的Kp=0.5、Ki=8/1024，
 * 只加入开环前馈，避免一次同时改变多个变量。
 */
#define WHEEL_SPEED_LEFT_KP_Q10                ((int32_t)512)
#define WHEEL_SPEED_LEFT_KI_Q10                ((int32_t)8)
#define WHEEL_SPEED_RIGHT_KP_Q10               ((int32_t)512)
#define WHEEL_SPEED_RIGHT_KI_Q10               ((int32_t)8)

/*
 * 积分项最大修正量。前馈承担主要基础PWM后，
 * 积分只负责负载、电压和电机差异修正。
 */
#define WHEEL_SPEED_INTEGRAL_LIMIT_PWM         ((int16_t)2500)

/*
 * 开环前馈：
 * PWM_ff = sign(target) ×
 *          (static_pwm + gain_q10 × abs(target_cps) / 1024)
 *
 * 系数由本次PWM=1000～5000的架空开环稳定数据线性拟合得到：
 * 左轮 pwm ≈ 124 + 0.9146 × cps
 * 右轮 pwm ≈ 183 + 0.9317 × cps
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
 * 等效测速窗口约20 ms，可降低5 ms内单个编码器计数带来的量化抖动。
 */
#define WHEEL_SPEED_MEASUREMENT_WINDOW         ((uint8_t)4U)

/*
 * 固定最小PWM补偿仍保持关闭。
 * V2使用前馈中的静态项跨越电机死区，不再依赖积分缓慢爬升。
 */
#define WHEEL_SPEED_LEFT_MIN_DRIVE_PWM         ((int16_t)0)
#define WHEEL_SPEED_RIGHT_MIN_DRIVE_PWM        ((int16_t)0)

#endif /* CHASSIS_CONFIG_H */
