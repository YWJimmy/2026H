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
 * 根据本次架空开环测试：
 * PWM=5000时左右轮约为5305/5150 count/s，
 * 对应约738/717 mm/s。
 *
 * 第一版把可请求速度限制为700 mm/s，
 * 对应约5032 count/s，避免长期不可达目标造成PWM饱和。
 */
#define CHASSIS_MAX_WHEEL_SPEED_MM_S           ((int32_t)700)
#define CHASSIS_MAX_WHEEL_SPEED_CPS            ((int32_t)5032)

/*
 * PI参数采用Q10定点：
 * 实际Kp = KP_Q10 / 1024
 * 实际Ki = KI_Q10 / 1024
 *
 * Ki表示每个标称5 ms周期的积分增量系数。
 * 当前仍为保守起始值，必须通过闭环日志继续整定。
 */
#define WHEEL_SPEED_LEFT_KP_Q10                ((int32_t)512)
#define WHEEL_SPEED_LEFT_KI_Q10                ((int32_t)8)
#define WHEEL_SPEED_RIGHT_KP_Q10               ((int32_t)512)
#define WHEEL_SPEED_RIGHT_KI_Q10               ((int32_t)8)

/*
 * 控制每5 ms执行一次，但测速使用最近4个控制周期的滑动窗口：
 * 等效测速窗口约20 ms，可降低5 ms内单个编码器计数带来的量化抖动。
 */
#define WHEEL_SPEED_MEASUREMENT_WINDOW         ((uint8_t)4U)

/*
 * 最小可靠启动PWM尚未通过100~500细分测试确定。
 * 当前不启用固定死区补偿，PI积分会自行跨过电机死区。
 */
#define WHEEL_SPEED_LEFT_MIN_DRIVE_PWM         ((int16_t)0)
#define WHEEL_SPEED_RIGHT_MIN_DRIVE_PWM        ((int16_t)0)

#endif /* CHASSIS_CONFIG_H */
