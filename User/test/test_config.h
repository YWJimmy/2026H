#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC          1U
#define TEST_MODE_LINE_SENSOR       2U
#define TEST_MODE_LINE_FOLLOW       3U

/* Default: verify BSP plus the unified 8-channel data layer. */
#define PROJECT_TEST_MODE           TEST_MODE_LINE_SENSOR

#endif /* TEST_CONFIG_H */
