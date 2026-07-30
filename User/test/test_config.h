#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC                  1U
#define TEST_MODE_LINE_SENSOR               2U
#define TEST_MODE_LINE_FOLLOW               3U
#define TEST_MODE_MOTOR_OPEN_LOOP           4U

/*
 * 保留原TEST_MODE_WHEEL_SPEED的数值5，兼容已有代码与历史配置。
 * 新代码明确区分mm/s和count/s两种轮速接口。
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
