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

/*
 * 新 UART 八路模块先运行原始通信测试：
 * - 不驱动电机；
 * - 不解释黑白极性；
 * - 不假定 S1 位于左侧或右侧。
 */
#define PROJECT_TEST_MODE               TEST_MODE_LINE_UART

#endif /* TEST_CONFIG_H */
