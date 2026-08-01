#ifndef LINE_FOLLOW_CONTROL_CONFIG_H
#define LINE_FOLLOW_CONTROL_CONFIG_H

#include <stdint.h>

/* 巡线外环PD采用Q10定点；保留V5稳定参数并恢复原目标速度。 */
#define LINE_FOLLOW_CONTROL_Q_SHIFT                  10
#define LINE_FOLLOW_CONTROL_KP_Q10                   ((int32_t)24)
#define LINE_FOLLOW_CONTROL_KD_Q10                   ((int32_t)6)

/* 误差滤波、中心死区和微分抑制。 */
#define LINE_FOLLOW_CONTROL_ERROR_MEDIAN_WINDOW      ((uint8_t)3U)
#define LINE_FOLLOW_CONTROL_ERROR_DEADBAND            ((int32_t)500)
#define LINE_FOLLOW_CONTROL_ERROR_DELTA_LIMIT         ((int32_t)2000)

/*
 * NORMAL状态基础速度：
 * 误差接近0时为360 mm/s；
 * |error|达到7000时线性降低到300 mm/s。
 */
#define LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S        ((int32_t)360)
#define LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S      ((int32_t)300)
#define LINE_FOLLOW_CONTROL_ERROR_FULL_SCALE         ((int32_t)7000)

/* 基础速度单独限速率，避免离散巡线误差造成直线/弯道速度跳变。 */
#define LINE_FOLLOW_CONTROL_BASE_ACCEL_SLEW_MM_S2    ((int32_t)180)
#define LINE_FOLLOW_CONTROL_BASE_DECEL_SLEW_MM_S2    ((int32_t)260)

/* 巡线过程中禁止反向，目标轮速限制为0～500 mm/s。 */
#define LINE_FOLLOW_CONTROL_MAX_WHEEL_SPEED_MM_S     ((int32_t)500)
#define LINE_FOLLOW_CONTROL_MAX_CORRECTION_MM_S      ((int32_t)110)

/* 转向修正先经过快速限速率，避免离散误差直接使差速反向跳变。 */
#define LINE_FOLLOW_CONTROL_CORRECTION_SLEW_MM_S2    ((int32_t)5000)

/* 启动后收到任意有效非空黑线帧即直接进入巡线，不要求黑线居中。 */

/* 丢线搜索降低速度并适当延长恢复窗口。 */
#define LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S   ((int32_t)100)
#define LINE_FOLLOW_CONTROL_LOST_TIMEOUT_MS          ((uint32_t)500U)

/* 全黑短时直行参数。 */
#define LINE_FOLLOW_CONTROL_ALL_BLACK_SPEED_MM_S     ((int32_t)100)
#define LINE_FOLLOW_CONTROL_ALL_BLACK_TIMEOUT_MS     ((uint32_t)250U)

/* KEY0=PE4，高电平按下；输入使用下拉。 */
#define LINE_FOLLOW_DRIVE_KEY_DEBOUNCE_MS            ((uint32_t)30U)

/* 调试输出周期。 */
#define LINE_FOLLOW_DRIVE_REPORT_PERIOD_MS           ((uint32_t)100U)

#endif /* LINE_FOLLOW_CONTROL_CONFIG_H */
