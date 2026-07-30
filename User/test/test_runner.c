#include "test_runner.h"

#include "test_config.h"

#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
#include "test_line_adc.h"
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
#include "test_line_sensor.h"
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_FOLLOW
#include "test_line_follow.h"
#else
#error "Unsupported PROJECT_TEST_MODE"
#endif

bool TestRunner_Init(void)
{
#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
    return Test_LineAdc_Init();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
    return Test_LineSensor_Init();
#else
    return Test_LineFollow_Init();
#endif
}

void TestRunner_Update(void)
{
#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
    Test_LineAdc_Update();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
    Test_LineSensor_Update();
#else
    Test_LineFollow_Update();
#endif
}

void TestRunner_Stop(void)
{
#if PROJECT_TEST_MODE == TEST_MODE_LINE_ADC
    Test_LineAdc_Stop();
#elif PROJECT_TEST_MODE == TEST_MODE_LINE_SENSOR
    Test_LineSensor_Stop();
#else
    Test_LineFollow_Stop();
#endif
}
