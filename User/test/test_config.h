#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC              1U
#define TEST_MODE_LINE_SENSOR           2U
#define TEST_MODE_LINE_FOLLOW           3U
#define TEST_MODE_MOTOR_OPEN_LOOP       4U
#define TEST_MODE_WHEEL_SPEED           5U
#define TEST_MODE_CHASSIS_STRAIGHT      6U
#define TEST_MODE_ENCODER               7U
#define TEST_MODE_VISION_UART           8U
#define TEST_MODE_BALL_BALANCE          9U

/*
 * 钢球平衡测试：
 * 舵机初始 1600us，P 控制使小球稳定在 cx=500。
 */
#define PROJECT_TEST_MODE               TEST_MODE_BALL_BALANCE

#endif /* TEST_CONFIG_H */