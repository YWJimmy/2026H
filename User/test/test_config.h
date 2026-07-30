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
 * 从main恢复的钢球平衡独立测试。
 * 使用20，避免与follow已有模式1～11冲突。
 */
#define TEST_MODE_BALL_BALANCE              20U

/*
 * 默认仍保持实际巡线驾驶模式，不改变底盘测试入口。
 * 编译钢球测试时改为TEST_MODE_BALL_BALANCE。
 */
#ifndef PROJECT_TEST_MODE
#define PROJECT_TEST_MODE                   TEST_MODE_LINE_FOLLOW_DRIVE
#endif

#endif /* TEST_CONFIG_H */
