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
#elif PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED
#include "test_wheel_speed.h"
#elif PROJECT_TEST_MODE == TEST_MODE_CHASSIS_STRAIGHT
#include "test_chassis_straight.h"
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
#elif PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED
    return Test_WheelSpeed_Init();
#else
    return Test_ChassisStraight_Init();
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
#elif PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED
    Test_WheelSpeed_Update();
#else
    Test_ChassisStraight_Update();
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
#elif PROJECT_TEST_MODE == TEST_MODE_WHEEL_SPEED
    Test_WheelSpeed_Stop();
#else
    Test_ChassisStraight_Stop();
#endif
}
