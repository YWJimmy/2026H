#ifndef LINE_SENSOR_CONFIG_H
#define LINE_SENSOR_CONFIG_H

#include <stdint.h>

/*
 * 八路巡线传感器配置文件
 * 修改硬件类型、安装方向或标定参数时，仅需修改本文件。
 */

/* ================= 巡线模块类型配置 ================= */
/* 选择巡线模块输入方式：ADC模拟八路或GPIO数字八路 */
#define LINE_SENSOR_BACKEND_ADC8             1U
#define LINE_SENSOR_BACKEND_GPIO8            2U
/* 当前使用ADC模拟八路巡线模块 */
#define LINE_SENSOR_BACKEND                  LINE_SENSOR_BACKEND_ADC8

/* ================= 传感器方向配置 ================= */
/* 定义八路传感器逻辑顺序：0为最左侧，7为最右侧 */
#define LINE_SENSOR_CHANNEL_MAP_INIT          \
{                                             \
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U           \
}
/* ================= ADC标定配置 ================= */
/* 白色区域ADC采样值，需要根据实际测试结果修改 */
#define LINE_SENSOR_WHITE_RAW_INIT            \
{                                             \
    2700U, 2700U, 2700U, 2700U,               \
    2700U, 2700U, 2700U, 2700U                \
}
/* 黑线区域ADC采样值，需要根据实际测试结果修改 */
#define LINE_SENSOR_BLACK_RAW_INIT            \
{                                             \
    3970U, 3970U, 3970U, 3970U,               \
    3970U, 3970U, 3970U, 3970U                \
}

/* ================= 黑白判断配置 ================= */
/* 黑白判断迟滞阈值，用于减少边界抖动 */
#define LINE_SENSOR_BLACK_ON_THRESHOLD        600U
#define LINE_SENSOR_WHITE_ON_THRESHOLD        400U
/* ================= 标定有效性配置 ================= */

/* 黑白ADC差值小于该值时，认为该通道标定无效 */
#define LINE_SENSOR_MIN_CALIBRATION_SPAN      16U

#endif /* LINE_SENSOR_CONFIG_H */
