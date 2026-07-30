#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC                  1U
#define TEST_MODE_LINE_SENSOR               2U
#define TEST_MODE_LINE_FOLLOW               3U
#define TEST_MODE_MOTOR_OPEN_LOOP           4U

/*
 * 保留历史TEST_MODE_WHEEL_SPEED名称，并将其明确作为mm/s模式别名。
 */
#define TEST_MODE_WHEEL_SPEED_MMPS          5U
#define TEST_MODE_WHEEL_SPEED               TEST_MODE_WHEEL_SPEED_MMPS
#define TEST_MODE_CHASSIS_STRAIGHT          6U
#define TEST_MODE_ENCODER                   7U
#define TEST_MODE_LINE_UART                 8U
#define TEST_MODE_VISION_UART               9U
#define TEST_MODE_WHEEL_SPEED_CPS           10U
#define TEST_MODE_LINE_FOLLOW_DRIVE         11U

/*
 * 实际巡线驾驶测试：
 * - 上电后电机保持不动；
 * - 第一次按PG15开始巡线；
 * - 第二次按PG15立即短路刹车停止；
 * - INVALID、丢线超时、全黑超时会自动停止；
 * - 自动停止后再次按PG15可重新开始。
 */
#define PROJECT_TEST_MODE                   TEST_MODE_LINE_FOLLOW_DRIVE

#endif /* TEST_CONFIG_H */
