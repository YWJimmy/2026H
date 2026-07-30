#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC              1U
#define TEST_MODE_LINE_SENSOR           2U
#define TEST_MODE_LINE_FOLLOW           3U
#define TEST_MODE_MOTOR_OPEN_LOOP       4U
#define TEST_MODE_WHEEL_SPEED           5U
#define TEST_MODE_CHASSIS_STRAIGHT      6U
#define TEST_MODE_ENCODER               7U
#define TEST_MODE_LINE_UART             8U
#define TEST_MODE_VISION_UART           9U

/*
 * 默认保留远端已确认的巡线闭环模式。
 * 验证 K230D 通信时切换为 TEST_MODE_VISION_UART；该模式不驱动电机，
 * 接收并解析 SB 帧，并通过 USART1 输出通信统计。
 */
#define TEST_MODE_WHEEL_SPEED_MMPS          5U
#define TEST_MODE_WHEEL_SPEED               TEST_MODE_WHEEL_SPEED_MMPS

#define TEST_MODE_CHASSIS_STRAIGHT          6U
#define TEST_MODE_ENCODER                   7U
#define TEST_MODE_LINE_UART                 8U
#define TEST_MODE_WHEEL_SPEED_CPS           9U

/*
 * PI闭环V2首次验证使用mm/s阶跃测试。
 * 测试完成后可恢复为：
 * #define PROJECT_TEST_MODE TEST_MODE_LINE_FOLLOW
 */
#define PROJECT_TEST_MODE                   TEST_MODE_WHEEL_SPEED_MMPS

#endif /* TEST_CONFIG_H */
