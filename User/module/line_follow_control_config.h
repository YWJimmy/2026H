#ifndef LINE_FOLLOW_CONTROL_CONFIG_H
#define LINE_FOLLOW_CONTROL_CONFIG_H

#include <stdint.h>

/*
 * 巡线外环全部使用整数。
 * PD系数采用Q10：
 * 实际系数 = 配置值 / 1024。
 */
#define LINE_FOLLOW_CONTROL_Q_SHIFT                  10
#define LINE_FOLLOW_CONTROL_KP_Q10                   ((int32_t)40)
#define LINE_FOLLOW_CONTROL_KD_Q10                   ((int32_t)16)

/*
 * NORMAL状态基础速度：
 * 误差接近0时为250 mm/s；
 * |error|达到7000时线性降低到120 mm/s。
 */
#define LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S        ((int32_t)250)
#define LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S      ((int32_t)120)
#define LINE_FOLLOW_CONTROL_ERROR_FULL_SCALE         ((int32_t)7000)

/*
 * 所有正常巡线轮速均限制在0～300 mm/s。
 * 不允许通过负轮速反向修正。
 */
#define LINE_FOLLOW_CONTROL_MAX_WHEEL_SPEED_MM_S     ((int32_t)300)
#define LINE_FOLLOW_CONTROL_MAX_CORRECTION_MM_S      ((int32_t)300)

/*
 * 丢线搜索：
 * 外侧轮160 mm/s，内侧轮0 mm/s；
 * 300 ms仍未恢复时停止并保持短路刹车。
 */
#define LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S   ((int32_t)160)
#define LINE_FOLLOW_CONTROL_LOST_TIMEOUT_MS          ((uint32_t)300U)

/*
 * 全黑：
 * 先按150 mm/s直行通过；
 * 持续超过200 ms时停止并保持短路刹车。
 */
#define LINE_FOLLOW_CONTROL_ALL_BLACK_SPEED_MM_S     ((int32_t)150)
#define LINE_FOLLOW_CONTROL_ALL_BLACK_TIMEOUT_MS     ((uint32_t)200U)

/*
 * 测试启动键PG15为低电平按下，使用30 ms非阻塞消抖。
 */
#define LINE_FOLLOW_DRIVE_KEY_DEBOUNCE_MS            ((uint32_t)30U)

/* 调试输出周期。 */
#define LINE_FOLLOW_DRIVE_REPORT_PERIOD_MS           ((uint32_t)100U)

#endif /* LINE_FOLLOW_CONTROL_CONFIG_H */
