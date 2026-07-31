#ifndef TASK_MENU_UI_H
#define TASK_MENU_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TASK_MENU_TASK_2_LAP_STOP = 2U,
    TASK_MENU_TASK_3_BALL_SEQUENCE,
    TASK_MENU_TASK_4_AB_HOLD,
    TASK_MENU_TASK_5_LAP_HOLD,
    TASK_MENU_TASK_6_LAP_TARGET
} TaskMenuTask_t;

typedef enum
{
    TASK_MENU_STATE_BOOT = 0,
    TASK_MENU_STATE_SELF_CHECK,
    TASK_MENU_STATE_SELECT,
    TASK_MENU_STATE_ARMED,
    TASK_MENU_STATE_STARTING,
    TASK_MENU_STATE_RUNNING,
    TASK_MENU_STATE_STOPPING,
    TASK_MENU_STATE_FINISHED,
    TASK_MENU_STATE_FAULT
} TaskMenuState_t;

typedef struct
{
    bool initialized;

    TaskMenuTask_t selected_task;
    TaskMenuState_t state;

    uint32_t elapsed_ms;
    uint32_t distance_mm;
    uint32_t warning_mask;
    uint32_t fault_code;
    uint32_t fault_detail;

    uint32_t selection_change_count;
    uint32_t start_request_count;
    uint32_t stop_request_count;
    uint32_t menu_request_count;
    uint32_t reset_request_count;
} TaskMenuStatus_t;

bool TaskMenuUi_Init(void);

/**
 * @brief 处理按键和OLED页面刷新。
 *
 * KEY_UP只在SELECT中切换任务，在ARMED中返回菜单。
 * 不实现长按。
 *
 * KEY0：
 * SELECT -> ARMED
 * ARMED -> 产生启动请求
 * STARTING/RUNNING -> 产生停止请求
 * FINISHED -> 产生返回菜单请求
 * FAULT -> 产生故障复位请求
 */
void TaskMenuUi_Process(void);

void TaskMenuUi_SetState(TaskMenuState_t state);
void TaskMenuUi_SetElapsedMs(uint32_t elapsed_ms);
void TaskMenuUi_SetDistanceMm(uint32_t distance_mm);
void TaskMenuUi_SetWarningMask(uint32_t warning_mask);
void TaskMenuUi_SetStatusText(const char *text);

/* Use the 128x64 XX.X timer page for active and finished tasks. */
void TaskMenuUi_SetFullscreenTimerEnabled(bool enabled);

void TaskMenuUi_SetFault(
    uint32_t fault_code,
    uint32_t fault_detail,
    const char *fault_name);

bool TaskMenuUi_TakeStartRequest(
    TaskMenuTask_t *task);

bool TaskMenuUi_TakeStopRequest(void);
bool TaskMenuUi_TakeMenuRequest(void);
bool TaskMenuUi_TakeResetRequest(void);

/* 保留旧接口名称，供现有测试代码兼容。 */
void TaskMenuUi_SetFinished(void);

TaskMenuTask_t TaskMenuUi_GetSelectedTask(void);
TaskMenuState_t TaskMenuUi_GetState(void);

bool TaskMenuUi_GetStatus(
    TaskMenuStatus_t *status);

const char *TaskMenuUi_TaskName(
    TaskMenuTask_t task);

const char *TaskMenuUi_StateName(
    TaskMenuState_t state);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MENU_UI_H */
