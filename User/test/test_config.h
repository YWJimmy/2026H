#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC              1U
#define TEST_MODE_LINE_SENSOR           2U
#define TEST_MODE_LINE_FOLLOW           3U

#define TEST_MODE_MOTOR_OPEN_LOOP       4U
#define TEST_MODE_WHEEL_SPEED           5U
#define TEST_MODE_CHASSIS_STRAIGHT      6U

/*
 * 安全默认值仍保持巡线数据测试，不会在上电后自动驱动电机。
 *
 * 底盘开发测试顺序：
 * 1. TEST_MODE_MOTOR_OPEN_LOOP
 * 2. TEST_MODE_WHEEL_SPEED
 * 3. TEST_MODE_CHASSIS_STRAIGHT
 */
#define PROJECT_TEST_MODE               TEST_MODE_LINE_SENSOR

#endif /* TEST_CONFIG_H */
