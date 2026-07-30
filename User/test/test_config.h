#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC              1U
#define TEST_MODE_LINE_SENSOR           2U
#define TEST_MODE_LINE_FOLLOW           3U
#define TEST_MODE_MOTOR_OPEN_LOOP       4U
#define TEST_MODE_WHEEL_SPEED           5U
#define TEST_MODE_CHASSIS_STRAIGHT      6U
#define TEST_MODE_ENCODER               7U

/*
 * 编码器独立测试：
 * 不驱动电机，手动转动车轮。
 */
#define PROJECT_TEST_MODE               TEST_MODE_MOTOR_OPEN_LOOP

#endif /* TEST_CONFIG_H */