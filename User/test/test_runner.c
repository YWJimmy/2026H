#include "test_runner.h"

#include "test_config.h"

#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
#include "test_line_adc.h"

#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
#include "test_line_sensor.h"

#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW
#include "test_line_follow.h"

#elif PROJECT_TEST_MODE == TEST_MODE_MOTOR_OPEN_LOOP
#include "test_motor_open_loop.h"

#elif ((PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_MMPS) || \
       (PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_CPS))
#include "test_wheel_speed.h"

#elif PROJECT_TEST_MODE == TEST_MODE_CHASSIS_STRAIGHT
#include "test_chassis_straight.h"

#elif PROJECT_TEST_MODE == TEST_MODE_ENCODER
#include "test_encoder.h"

#elif PROJECT_TEST_MODE == TEST_MODE_LINE_UART
#include "test_line_uart.h"

#elif PROJECT_TEST_MODE == TEST_MODE_VISION_UART
#include "test_vision_uart.h"

#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW_DRIVE
#include "test_line_follow_drive.h"

#else
#error "Unsupported PROJECT_TEST_MODE"
#endif

bool TestRunner_Init(void)
{
#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
    return Test_LineAdc_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
    return Test_LineSensor_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW
    return Test_LineFollow_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_MOTOR_OPEN_LOOP
    return Test_MotorOpenLoop_Init();
#elif ((PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_MMPS) || \
       (PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_CPS))
    return Test_WheelSpeed_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_CHASSIS_STRAIGHT
    return Test_ChassisStraight_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_ENCODER
    return Test_Encoder_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_UART
    return Test_LineUart_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_VISION_UART
    return Test_VisionUart_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW_DRIVE
    return Test_LineFollowDrive_Init();
#else
    return false;
#endif
}

void TestRunner_Update(void)
{
#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
    Test_LineAdc_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
    Test_LineSensor_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW
    Test_LineFollow_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_MOTOR_OPEN_LOOP
    Test_MotorOpenLoop_Update();
#elif ((PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_MMPS) || \
       (PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_CPS))
    Test_WheelSpeed_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_CHASSIS_STRAIGHT
    Test_ChassisStraight_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_ENCODER
    Test_Encoder_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_UART
    Test_LineUart_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_VISION_UART
    Test_VisionUart_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW_DRIVE
    Test_LineFollowDrive_Update();
#endif
}

void TestRunner_Stop(void)
{
#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
    Test_LineAdc_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
    Test_LineSensor_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW
    Test_LineFollow_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_MOTOR_OPEN_LOOP
    Test_MotorOpenLoop_Stop();
#elif ((PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_MMPS) || \
       (PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED_CPS))
    Test_WheelSpeed_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_CHASSIS_STRAIGHT
    Test_ChassisStraight_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_ENCODER
    /* 编码器测试不驱动输出，无需主动停止。 */
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_UART
    Test_LineUart_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_VISION_UART
    Test_VisionUart_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW_DRIVE
    Test_LineFollowDrive_Stop();
#endif
}
