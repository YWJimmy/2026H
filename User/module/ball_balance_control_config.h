#ifndef BALL_BALANCE_CONTROL_CONFIG_H
#define BALL_BALANCE_CONTROL_CONFIG_H

#include <stdint.h>

/*
 * 视觉坐标系沿用f426底版的1280x960坐标。
 * 目标中心位置保持为x=500。
 */
#define BALL_BALANCE_TARGET_CX                     ((int32_t)500)

/*
 * 47d12c9钢球控制范围：
 * - 1600 us：水平稳定位置；
 * - 正常推球：1400～1800 us；
 * - 反向制动：1300～1900 us。
 */
#define BALL_BALANCE_SERVO_CENTER_US               ((uint16_t)1600U)
#define BALL_BALANCE_SERVO_PUSH_MIN_US             ((uint16_t)1400U)
#define BALL_BALANCE_SERVO_PUSH_MAX_US             ((uint16_t)1800U)
#define BALL_BALANCE_SERVO_BRAKE_MIN_US            ((uint16_t)1300U)
#define BALL_BALANCE_SERVO_BRAKE_MAX_US            ((uint16_t)1900U)

/* f426整数Q10推球比例控制。 */
#define BALL_BALANCE_PUSH_KP_Q10                   ((int32_t)1024)
#define BALL_BALANCE_GAIN_Q_SHIFT                  10

/* 47d12c9策略：|error| < |speed| * 5时开始反向制动。 */
#define BALL_BALANCE_BRAKE_DISTANCE_GAIN           ((int32_t)5)

/* f426制动偏移配置：60 + |speed| * 3 / 2，最大120 us。 */
#define BALL_BALANCE_BRAKE_BASE_US                 ((int32_t)60)
#define BALL_BALANCE_BRAKE_SPEED_NUMERATOR         ((int32_t)3)
#define BALL_BALANCE_BRAKE_SPEED_DENOMINATOR       ((int32_t)2)
#define BALL_BALANCE_BRAKE_MAX_US                  ((int32_t)120)

/* 47d12c9低速卡滞判断与逐帧增力。 */
#define BALL_BALANCE_STUCK_SPEED_PX                ((int32_t)3)
#define BALL_BALANCE_STUCK_BOOST_US                ((uint32_t)5U)

/* |error|不超过30像素时保持水平。 */
#define BALL_BALANCE_DEADZONE_PX                   ((int32_t)30)

/* 非零推球命令的最小舵机偏移。 */
#define BALL_BALANCE_MIN_STEP_US                   ((int32_t)20)

/* 连续丢失目标10帧后舵机回中。 */
#define BALL_BALANCE_LOST_TIMEOUT_FRAMES           ((uint32_t)10U)

#endif /* BALL_BALANCE_CONTROL_CONFIG_H */
