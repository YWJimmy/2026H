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
    TASK_MENU_STATE_SELECT = 0,
    TASK_MENU_STATE_ARMED,
    TASK_MENU_STATE_RUNNING,
    TASK_MENU_STATE_FINISHED
} TaskMenuState_t;

typedef struct
{
    bool initialized;
    TaskMenuTask_t selected_task;
    TaskMenuState_t state;
    uint32_t selection_change_count;
    uint32_t start_request_count;
    uint32_t stop_request_count;
} TaskMenuStatus_t;

/**
 * @brief 初始化任务2～6菜单。
 */
bool TaskMenuUi_Init(void);

/**
 * @brief 处理按键并在状态变化时更新OLED帧缓冲区。
 *
 * SELECT：
 * - KEY_UP：循环选择任务2～6
 * - KEY0：进入ARMED
 *
 * ARMED：
 * - KEY_UP：返回SELECT
 * - KEY0：产生启动请求并进入RUNNING
 *
 * RUNNING：
 * - KEY0：产生停止请求并进入FINISHED
 *
 * FINISHED：
 * - KEY0：返回SELECT
 */
void TaskMenuUi_Process(void);

bool TaskMenuUi_TakeStartRequest(
    TaskMenuTask_t *task);

bool TaskMenuUi_TakeStopRequest(void);

/**
 * @brief 由未来正式任务状态机通知自然完成。
 */
void TaskMenuUi_SetFinished(void);

/**
 * @brief 显示任务结果和任务用时。
 */
/**
 * @brief 更新任务3运行中的自动计时显示。
 *
 * 仅当十分之一秒发生变化时重绘OLED，避免无意义的I2C刷新。
 */
void TaskMenuUi_SetRunningElapsedMs(
    uint32_t elapsed_ms,
    bool timer_running);

void TaskMenuUi_SetFinishedResult(
    bool passed,
    uint32_t elapsed_ms);

TaskMenuTask_t TaskMenuUi_GetSelectedTask(void);
TaskMenuState_t TaskMenuUi_GetState(void);
bool TaskMenuUi_GetStatus(TaskMenuStatus_t *status);

const char *TaskMenuUi_TaskName(TaskMenuTask_t task);
const char *TaskMenuUi_StateName(TaskMenuState_t state);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MENU_UI_H */
