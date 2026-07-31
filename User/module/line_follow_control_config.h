#ifndef LINE_FOLLOW_CONTROL_CONFIG_H
#define LINE_FOLLOW_CONTROL_CONFIG_H

#include <stdint.h>

/* 巡线外环PD采用Q10定点。 */
#define LINE_FOLLOW_CONTROL_Q_SHIFT                  10
#define LINE_FOLLOW_CONTROL_KP_Q10                   ((int32_t)40)
#define LINE_FOLLOW_CONTROL_KD_Q10                   ((int32_t)16)

/*
 * NORMAL状态基础速度：
 * 误差接近0时为360 mm/s；
 * |error|达到7000时线性降低到120 mm/s。
 */
#define LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S        ((int32_t)360)
#define LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S      ((int32_t)120)
#define LINE_FOLLOW_CONTROL_ERROR_FULL_SCALE         ((int32_t)7000)

/* 巡线过程中禁止反向，目标轮速限制为0～500 mm/s。 */
#define LINE_FOLLOW_CONTROL_MAX_WHEEL_SPEED_MM_S     ((int32_t)500)
#define LINE_FOLLOW_CONTROL_MAX_CORRECTION_MM_S      ((int32_t)500)

/* 丢线搜索参数。 */
#define LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S   ((int32_t)160)
#define LINE_FOLLOW_CONTROL_LOST_TIMEOUT_MS          ((uint32_t)300U)

/* 全黑短时直行参数。 */
#define LINE_FOLLOW_CONTROL_ALL_BLACK_SPEED_MM_S     ((int32_t)150)
#define LINE_FOLLOW_CONTROL_ALL_BLACK_TIMEOUT_MS     ((uint32_t)200U)

/* KEY0=PE4，高电平按下；输入使用下拉。 */
#define LINE_FOLLOW_DRIVE_KEY_DEBOUNCE_MS            ((uint32_t)30U)

/* 调试输出周期。 */
#define LINE_FOLLOW_DRIVE_REPORT_PERIOD_MS           ((uint32_t)100U)

#endif /* LINE_FOLLOW_CONTROL_CONFIG_H */
