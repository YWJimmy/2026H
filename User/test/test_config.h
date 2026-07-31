#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC                  1U
#define TEST_MODE_LINE_SENSOR               2U
#define TEST_MODE_LINE_FOLLOW               3U
#define TEST_MODE_MOTOR_OPEN_LOOP           4U
#define TEST_MODE_WHEEL_SPEED_MMPS          5U
#define TEST_MODE_WHEEL_SPEED               TEST_MODE_WHEEL_SPEED_MMPS
#define TEST_MODE_CHASSIS_STRAIGHT          6U
#define TEST_MODE_ENCODER                   7U
#define TEST_MODE_LINE_UART                 8U
#define TEST_MODE_VISION_UART               9U
#define TEST_MODE_WHEEL_SPEED_CPS           10U
#define TEST_MODE_LINE_FOLLOW_DRIVE         11U
#define TEST_MODE_CHASSIS_RAMP              12U
#define TEST_MODE_OLED_KEY                  13U

#define TEST_MODE_BALL_BALANCE              20U

/*
 * 钢球往返测试
 */
#define PROJECT_TEST_MODE                   TEST_MODE_BALL_BALANCE

#endif /* TEST_CONFIG_H */
