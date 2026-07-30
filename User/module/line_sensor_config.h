#ifndef LINE_SENSOR_CONFIG_H
#define LINE_SENSOR_CONFIG_H

#include <stdint.h>

/* ================= 巡线模块类型配置 ================= */
#define LINE_SENSOR_BACKEND_ADC8             1U
#define LINE_SENSOR_BACKEND_GPIO8            2U
#define LINE_SENSOR_BACKEND_UART8            3U

/* 当前正式后端切换为新 UART 八路模块。 */
#define LINE_SENSOR_BACKEND                  LINE_SENSOR_BACKEND_UART8

/* ================= ADC 旧后端配置 ================= */
#define LINE_SENSOR_ADC_CHANNEL_MAP_INIT      \
{                                             \
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U           \
}

#define LINE_SENSOR_WHITE_RAW_INIT            \
{                                             \
    3100U, 3100U, 3100U, 3100U,               \
    3100U, 3100U, 3100U, 3100U                \
}

#define LINE_SENSOR_BLACK_RAW_INIT            \
{                                             \
    3700U, 3700U, 3700U, 3700U,               \
    3700U, 3700U, 3700U, 3700U                \
}

#define LINE_SENSOR_BLACK_ON_THRESHOLD       600U
#define LINE_SENSOR_WHITE_ON_THRESHOLD       400U
#define LINE_SENSOR_MIN_CALIBRATION_SPAN      16U

/* ================= UART 新后端配置 ================= */
/*
 * 必须完成逐路实测后才能置 1：
 * 1. 确认原始位 1/0 哪个表示检测到黑线；
 * 2. 确认 S1 在车辆左侧还是右侧。
 *
 * 为 0 时仍可读取 raw[]，但 valid_mask 固定为 0，
 * line_follow 会进入 INVALID，避免未确认配置时驱动车辆。
 */
#define LINE_SENSOR_UART_CONFIG_CONFIRMED    0U

/* 检测到黑线时模块原始位值：实测后设为 0U 或 1U。 */
#define LINE_SENSOR_UART_BLACK_LEVEL         1U

/*
 * 逻辑通道 0~7 始终定义为车辆最左到最右。
 * 数组元素是协议物理通道索引：0=S1，7=S8。
 *
 * 若 S1 在最左：{0,1,2,3,4,5,6,7}
 * 若 S1 在最右：{7,6,5,4,3,2,1,0}
 */
#define LINE_SENSOR_UART_CHANNEL_MAP_INIT     \
{                                             \
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U           \
}

#if LINE_SENSOR_UART_BLACK_LEVEL > 1U
#error "LINE_SENSOR_UART_BLACK_LEVEL must be 0U or 1U"
#endif

#endif /* LINE_SENSOR_CONFIG_H */
