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
 * K230D 视觉串口安全测试：
 * - 不驱动电机；
 * - 接收并解析 SB 帧；
 * - 通过 USART1 调试串口输出通信统计。
 */
#define PROJECT_TEST_MODE               TEST_MODE_VISION_UART

#endif /* TEST_CONFIG_H */
