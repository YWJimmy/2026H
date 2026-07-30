#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

/* 用户实测与确认的底盘参数。 */
#define CHASSIS_WHEEL_DIAMETER_MM          ((int32_t)65)
#define CHASSIS_TRACK_WIDTH_MM             ((int32_t)210)

/*
 * 圆周长 = π × 65 mm ≈ 204.2035 mm。
 * 使用微米整数保存，避免运行时浮点运算。
 */
#define CHASSIS_WHEEL_CIRCUMFERENCE_UM     ((int32_t)204204)

/* 5 ms，200 Hz标称控制周期。 */
#define CHASSIS_CONTROL_PERIOD_MS          ((uint16_t)5U)

/*
 * 主循环若超过该时间没有执行控制更新，则视为严重超时：
 * 本周期短路刹车、重置PI并重新同步编码器。
 */
#define CHASSIS_TIMING_OVERRUN_MS          ((uint16_t)25U)

/* 初期安全限制，低于TIM9硬件最大值8399。 */
#define CHASSIS_PWM_LIMIT                  ((int16_t)5000)

/* 底盘API允许的单轮速度范围。 */
#define CHASSIS_MAX_WHEEL_SPEED_MM_S       ((int32_t)1200)

/*
 * PI参数采用Q10定点：
 * 实际Kp = WHEEL_SPEED_KP_Q10 / 1024
 * 实际Ki = WHEEL_SPEED_KI_Q10 / 1024
 *
 * Ki表示每个标称5 ms周期的积分增量系数。
 * 当前为保守初值，必须通过开环和闭环测试继续整定。
 */
#define WHEEL_SPEED_LEFT_KP_Q10            ((int32_t)512)
#define WHEEL_SPEED_LEFT_KI_Q10            ((int32_t)8)
#define WHEEL_SPEED_RIGHT_KP_Q10           ((int32_t)512)
#define WHEEL_SPEED_RIGHT_KI_Q10           ((int32_t)8)

/*
 * 电机最小可靠启动PWM尚未实测，当前不启用死区补偿。
 * 完成TEST_MODE_MOTOR_OPEN_LOOP后分别填入实测值。
 */
#define WHEEL_SPEED_LEFT_MIN_DRIVE_PWM     ((int16_t)0)
#define WHEEL_SPEED_RIGHT_MIN_DRIVE_PWM    ((int16_t)0)

#endif /* CHASSIS_CONFIG_H */
