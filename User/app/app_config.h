#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* 顶层状态机时间参数。 */
#define APP_BOOT_DELAY_MS                   ((uint32_t)150U)
#define APP_SELF_CHECK_TIMEOUT_MS           ((uint32_t)1500U)
#define APP_UI_RUNTIME_REFRESH_MS            ((uint32_t)100U)
#define APP_DEBUG_REPORT_PERIOD_MS           ((uint32_t)1000U)

/*
 * 当前交付仅搭建主状态机骨架。
 * 任务2～6的真实运动逻辑尚未接入，app_task_port.c保持安全空实现。
 */
#define APP_TASK_PORT_PLACEHOLDER            1U

/* 任务骨架启动和停车状态演示时间。 */
#define APP_TASK_START_SETTLE_MS             ((uint32_t)20U)
#define APP_TASK_STOP_SETTLE_MS              ((uint32_t)100U)
#define APP_TASK_STOP_TIMEOUT_MS             ((uint32_t)1500U)

/* 已确认的正式任务公共参数，供后续任务实现直接使用。 */
#define APP_ROUTE_SENSOR_FORWARD_OFFSET_MM   ((int32_t)85)
#define APP_TASK3_PLUS_HOLD_MS               ((uint32_t)500U)
#define APP_TASK6_TARGET_MIN_MM              ((int32_t)-125)
#define APP_TASK6_TARGET_MAX_MM              ((int32_t)125)

/* 自检警告位。 */
#define APP_WARNING_NONE                     ((uint32_t)0U)
#define APP_WARNING_OLED_OFFLINE             ((uint32_t)(1UL << 0))
#define APP_WARNING_TASK_PLACEHOLDER         ((uint32_t)(1UL << 1))

#endif /* APP_CONFIG_H */
