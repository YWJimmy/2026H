#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC          1U      //ADC数据是否采对
#define TEST_MODE_LINE_SENSOR       2U      //黑白数据是否转换正确
#define TEST_MODE_LINE_FOLLOW       3U      //巡线偏差是否计算正确

/* Default: verify BSP plus the unified 8-channel data layer. */
#define PROJECT_TEST_MODE           TEST_MODE_LINE_SENSOR

#endif /* TEST_CONFIG_H */
