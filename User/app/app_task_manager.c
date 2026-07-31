#include "app_task_manager.h"

#include "app_config.h"
#include "app_task_port.h"
#include "stm32f4xx_hal.h"

#include <stddef.h>

static AppTaskManagerStatus_t s_status;

static bool AppTaskManager_Elapsed(
    uint32_t now_ms,
    uint32_t start_ms,
    uint32_t duration_ms)
{
    return ((uint32_t)(now_ms - start_ms) >=
            duration_ms);
}

static void AppTaskManager_SetState(
    AppTaskManagerState_t state,
    uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_timestamp_ms = now_ms;
}

bool AppTaskManager_Init(void)
{
    s_status.initialized = false;
    s_status.state = APP_TASK_MANAGER_IDLE;
    s_status.task = TASK_MENU_TASK_2_LAP_STOP;
    s_status.stop_reason = APP_TASK_STOP_NONE;
    s_status.start_timestamp_ms = 0U;
    s_status.state_timestamp_ms = HAL_GetTick();
    s_status.elapsed_ms = 0U;
    s_status.update_count = 0U;
    s_status.fault_detail = 0U;

    if (!AppTaskPort_Init())
    {
        AppTaskPort_ForceSafeStop();
        return false;
    }

    s_status.initialized = true;
    return true;
}

bool AppTaskManager_Start(
    TaskMenuTask_t task,
    uint32_t start_timestamp_ms)
{
    if ((!s_status.initialized) ||
        ((s_status.state != APP_TASK_MANAGER_IDLE) &&
         (s_status.state != APP_TASK_MANAGER_FINISHED)))
    {
        return false;
    }

    AppTaskPort_Reset();

    if (!AppTaskPort_Start(
            task,
            start_timestamp_ms))
    {
        AppTaskPort_ForceSafeStop();
        s_status.fault_detail =
            AppTaskPort_GetFaultDetail();
        AppTaskManager_SetState(
            APP_TASK_MANAGER_FAULT,
            HAL_GetTick());
        return false;
    }

    s_status.task = task;
    s_status.stop_reason = APP_TASK_STOP_NONE;
    s_status.start_timestamp_ms =
        start_timestamp_ms;
    s_status.elapsed_ms = 0U;
    s_status.update_count = 0U;
    s_status.fault_detail = 0U;

    AppTaskManager_SetState(
        APP_TASK_MANAGER_STARTING,
        start_timestamp_ms);

    return true;
}

void AppTaskManager_Update(void)
{
    uint32_t now_ms;
    AppTaskPortResult_t result;

    if (!s_status.initialized)
    {
        return;
    }

    AppTaskPort_ProcessInputs();

    now_ms = HAL_GetTick();
    s_status.update_count++;

    if ((s_status.state == APP_TASK_MANAGER_STARTING) ||
        (s_status.state == APP_TASK_MANAGER_RUNNING) ||
        ((s_status.state == APP_TASK_MANAGER_STOPPING) &&
         (s_status.stop_reason != APP_TASK_STOP_NATURAL)))
    {
        s_status.elapsed_ms =
            (uint32_t)(now_ms -
                       s_status.start_timestamp_ms);
    }

    switch (s_status.state)
    {
        case APP_TASK_MANAGER_STARTING:
            if (AppTaskManager_Elapsed(
                    now_ms,
                    s_status.state_timestamp_ms,
                    APP_TASK_START_SETTLE_MS))
            {
                AppTaskManager_SetState(
                    APP_TASK_MANAGER_RUNNING,
                    now_ms);
            }
            break;

        case APP_TASK_MANAGER_RUNNING:
            result = AppTaskPort_Update(
                s_status.task,
                now_ms);

            if (result == APP_TASK_PORT_RESULT_FINISHED)
            {
                s_status.stop_reason =
                    APP_TASK_STOP_NATURAL;

                if (!AppTaskPort_RequestStop(
                        s_status.task))
                {
                    AppTaskPort_ForceSafeStop();
                    s_status.fault_detail =
                        AppTaskPort_GetFaultDetail();
                    AppTaskManager_SetState(
                        APP_TASK_MANAGER_FAULT,
                        now_ms);
                    break;
                }

                AppTaskManager_SetState(
                    APP_TASK_MANAGER_STOPPING,
                    now_ms);
            }
            else if (result ==
                     APP_TASK_PORT_RESULT_FAULT)
            {
                s_status.stop_reason =
                    APP_TASK_STOP_FAULT;
                s_status.fault_detail =
                    AppTaskPort_GetFaultDetail();

                AppTaskPort_ForceSafeStop();
                AppTaskManager_SetState(
                    APP_TASK_MANAGER_FAULT,
                    now_ms);
            }
            break;

        case APP_TASK_MANAGER_STOPPING:
            result = AppTaskPort_Update(
                s_status.task,
                now_ms);

            if (result == APP_TASK_PORT_RESULT_FAULT)
            {
                s_status.stop_reason =
                    APP_TASK_STOP_FAULT;
                s_status.fault_detail =
                    AppTaskPort_GetFaultDetail();
                AppTaskPort_ForceSafeStop();
                AppTaskManager_SetState(
                    APP_TASK_MANAGER_FAULT,
                    now_ms);
            }
            else if (AppTaskPort_IsStopped(
                    s_status.task) &&
                AppTaskManager_Elapsed(
                    now_ms,
                    s_status.state_timestamp_ms,
                    APP_TASK_STOP_SETTLE_MS))
            {
                AppTaskManager_SetState(
                    APP_TASK_MANAGER_FINISHED,
                    now_ms);
            }
            else if (AppTaskManager_Elapsed(
                         now_ms,
                         s_status.state_timestamp_ms,
                         APP_TASK_STOP_TIMEOUT_MS))
            {
                s_status.fault_detail = 1U;
                AppTaskPort_ForceSafeStop();
                AppTaskManager_SetState(
                    APP_TASK_MANAGER_FAULT,
                    now_ms);
            }
            break;

        case APP_TASK_MANAGER_IDLE:
        case APP_TASK_MANAGER_FINISHED:
        case APP_TASK_MANAGER_FAULT:
        default:
            break;
    }
}

bool AppTaskManager_RequestStop(
    AppTaskStopReason_t reason)
{
    uint32_t now_ms;

    if (!s_status.initialized)
    {
        return false;
    }

    if (s_status.state == APP_TASK_MANAGER_STOPPING)
    {
        return true;
    }

    if ((s_status.state != APP_TASK_MANAGER_STARTING) &&
        (s_status.state != APP_TASK_MANAGER_RUNNING))
    {
        return false;
    }

    now_ms = HAL_GetTick();

    if (!AppTaskPort_RequestStop(s_status.task))
    {
        AppTaskPort_ForceSafeStop();
        s_status.fault_detail =
            AppTaskPort_GetFaultDetail();
        AppTaskManager_SetState(
            APP_TASK_MANAGER_FAULT,
            now_ms);
        return false;
    }

    s_status.stop_reason = reason;
    AppTaskManager_SetState(
        APP_TASK_MANAGER_STOPPING,
        now_ms);

    return true;
}

void AppTaskManager_Reset(void)
{
    AppTaskPort_Reset();

    s_status.state = APP_TASK_MANAGER_IDLE;
    s_status.task = TASK_MENU_TASK_2_LAP_STOP;
    s_status.stop_reason = APP_TASK_STOP_NONE;
    s_status.start_timestamp_ms = 0U;
    s_status.state_timestamp_ms = HAL_GetTick();
    s_status.elapsed_ms = 0U;
    s_status.update_count = 0U;
    s_status.fault_detail = 0U;
}

bool AppTaskManager_IsRunning(void)
{
    return ((s_status.state ==
             APP_TASK_MANAGER_STARTING) ||
            (s_status.state ==
             APP_TASK_MANAGER_RUNNING));
}

bool AppTaskManager_IsFinished(void)
{
    return (s_status.state ==
            APP_TASK_MANAGER_FINISHED);
}

bool AppTaskManager_IsFaulted(void)
{
    return (s_status.state ==
            APP_TASK_MANAGER_FAULT);
}

bool AppTaskManager_GetStatus(
    AppTaskManagerStatus_t *status)
{
    if ((!s_status.initialized) ||
        (status == NULL))
    {
        return false;
    }

    *status = s_status;
    return true;
}

const char *AppTaskManager_StateName(
    AppTaskManagerState_t state)
{
    switch (state)
    {
        case APP_TASK_MANAGER_IDLE:
            return "IDLE";

        case APP_TASK_MANAGER_STARTING:
            return "STARTING";

        case APP_TASK_MANAGER_RUNNING:
            return "RUNNING";

        case APP_TASK_MANAGER_STOPPING:
            return "STOPPING";

        case APP_TASK_MANAGER_FINISHED:
            return "FINISHED";

        case APP_TASK_MANAGER_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

const char *AppTaskManager_StopReasonName(
    AppTaskStopReason_t reason)
{
    switch (reason)
    {
        case APP_TASK_STOP_NONE:
            return "NONE";

        case APP_TASK_STOP_USER:
            return "USER";

        case APP_TASK_STOP_NATURAL:
            return "NATURAL";

        case APP_TASK_STOP_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

const char *AppTaskManager_GetPhaseText(void)
{
    return AppTaskPort_GetPhaseText();
}
