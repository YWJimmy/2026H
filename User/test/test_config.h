#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC                  1U
#define TEST_MODE_LINE_SENSOR               2U
#define TEST_MODE_LINE_FOLLOW               3U
#define TEST_MODE_MOTOR_OPEN_LOOP           4U
#define TEST_MODE_WHEEL_SPEED_MMPS          5U
#define TEST_MODE_CHASSIS_STRAIGHT          6U
#define TEST_MODE_ENCODER                   7U
#define TEST_MODE_WHEEL_SPEED_CPS           8U

/* 兼容旧名称：旧TEST_MODE_WHEEL_SPEED等同于毫米每秒接口测试。 */
#define TEST_MODE_WHEEL_SPEED               TEST_MODE_WHEEL_SPEED_MMPS

/*
 * 默认进入mm/s单位的PI闭环测试。
 * 首次运行必须架空左右车轮。
 *
 * 也可切换为：
 * TEST_MODE_WHEEL_SPEED_CPS
 */
#define PROJECT_TEST_MODE                   TEST_MODE_WHEEL_SPEED_MMPS

#endif /* TEST_CONFIG_H */
