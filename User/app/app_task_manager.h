#ifndef APP_TASK_MANAGER_H
#define APP_TASK_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "task_menu_ui.h"

typedef enum
{
    APP_TASK_MANAGER_IDLE = 0,
    APP_TASK_MANAGER_STARTING,
    APP_TASK_MANAGER_RUNNING,
    APP_TASK_MANAGER_STOPPING,
    APP_TASK_MANAGER_FINISHED,
    APP_TASK_MANAGER_FAULT
} AppTaskManagerState_t;

typedef enum
{
    APP_TASK_STOP_NONE = 0,
    APP_TASK_STOP_USER,
    APP_TASK_STOP_NATURAL,
    APP_TASK_STOP_FAULT
} AppTaskStopReason_t;

typedef struct
{
    bool initialized;

    AppTaskManagerState_t state;
    TaskMenuTask_t task;
    AppTaskStopReason_t stop_reason;

    uint32_t start_timestamp_ms;
    uint32_t state_timestamp_ms;
    uint32_t elapsed_ms;
    uint32_t update_count;
    uint32_t fault_detail;
} AppTaskManagerStatus_t;

bool AppTaskManager_Init(void);
bool AppTaskManager_Start(
    TaskMenuTask_t task,
    uint32_t start_timestamp_ms);

void AppTaskManager_Update(void);

bool AppTaskManager_RequestStop(
    AppTaskStopReason_t reason);

void AppTaskManager_Reset(void);

bool AppTaskManager_IsRunning(void);
bool AppTaskManager_IsFinished(void);
bool AppTaskManager_IsFaulted(void);

bool AppTaskManager_GetStatus(
    AppTaskManagerStatus_t *status);

const char *AppTaskManager_StateName(
    AppTaskManagerState_t state);

const char *AppTaskManager_StopReasonName(
    AppTaskStopReason_t reason);

const char *AppTaskManager_GetPhaseText(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_MANAGER_H */
