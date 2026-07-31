#include "main_app.h"

#include "app_config.h"
#include "app_task_manager.h"
#include "app_task_port.h"

#include "bsp_debug_uart.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "distance_tracker.h"

#include "stm32f4xx_hal.h"

#include <stddef.h>

static MainAppStatus_t s_status;

static AppState_t s_last_logged_state =
    APP_STATE_FAULT;

static uint32_t s_last_debug_report_ms = 0U;
static bool s_key_ready = false;
static bool s_oled_ready = false;
static bool s_ui_ready = false;
static bool s_task_manager_ready = false;

static bool MainApp_Elapsed(
    uint32_t now_ms,
    uint32_t start_ms,
    uint32_t duration_ms)
{
    return ((uint32_t)(now_ms - start_ms) >=
            duration_ms);
}

const char *MainApp_StateName(
    AppState_t state)
{
    switch (state)
    {
        case APP_STATE_BOOT:
            return "BOOT";

        case APP_STATE_SELF_CHECK:
            return "SELF_CHECK";

        case APP_STATE_MENU:
            return "MENU";

        case APP_STATE_ARMED:
            return "ARMED";

        case APP_STATE_STARTING:
            return "STARTING";

        case APP_STATE_RUNNING:
            return "RUNNING";

        case APP_STATE_STOPPING:
            return "STOPPING";

        case APP_STATE_FINISHED:
            return "FINISHED";

        case APP_STATE_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

static void MainApp_SetState(
    AppState_t state,
    uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_timestamp_ms = now_ms;

    if (!s_ui_ready)
    {
        return;
    }

    switch (state)
    {
        case APP_STATE_BOOT:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_BOOT);
            break;

        case APP_STATE_SELF_CHECK:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_SELF_CHECK);
            break;

        case APP_STATE_MENU:
            TaskMenuUi_SetElapsedMs(0U);
            TaskMenuUi_SetStatusText(
                "SAFE SKELETON");
            TaskMenuUi_SetState(
                TASK_MENU_STATE_SELECT);
            break;

        case APP_STATE_ARMED:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_ARMED);
            break;

        case APP_STATE_STARTING:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_STARTING);
            break;

        case APP_STATE_RUNNING:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_RUNNING);
            break;

        case APP_STATE_STOPPING:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_STOPPING);
            break;

        case APP_STATE_FINISHED:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_FINISHED);
            break;

        case APP_STATE_FAULT:
        default:
            TaskMenuUi_SetState(
                TASK_MENU_STATE_FAULT);
            break;
    }
}

static void MainApp_EnterFault(
    AppFaultCode_t code,
    uint32_t detail)
{
    uint32_t now_ms = HAL_GetTick();

    AppTaskPort_ForceSafeStop();
    AppFault_Raise(code, detail);

    s_status.fault_code = code;
    s_status.fault_detail = detail;

    MainApp_SetState(
        APP_STATE_FAULT,
        now_ms);

    if (s_ui_ready)
    {
        TaskMenuUi_SetFault(
            (uint32_t)code,
            AppFault_Name(code));
    }
    else if (s_oled_ready)
    {
        BSP_Oled_Clear();
        BSP_Oled_DrawString(0U, 0U, "FAULT");
        BSP_Oled_DrawString(0U, 2U, AppFault_Name(code));
        BSP_Oled_DrawString(0U, 4U, "CODE:");
        BSP_Oled_DrawU32(36U, 4U, (uint32_t)code);
    }

    (void)BSP_Debug_Printf(
        "APP,FAULT=%s,CODE=%u,DETAIL=%lu\r\n",
        AppFault_Name(code),
        (unsigned int)code,
        (unsigned long)detail);
}

static void MainApp_UpdateTaskStatus(void)
{
    AppTaskManagerStatus_t task_status;

    if (!AppTaskManager_GetStatus(
            &task_status))
    {
        return;
    }

    s_status.elapsed_ms =
        task_status.elapsed_ms;

    TaskMenuUi_SetElapsedMs(
        task_status.elapsed_ms);

    TaskMenuUi_SetStatusText(
        AppTaskManager_GetPhaseText());
}

static void MainApp_UpdateDistance(void)
{
    DistanceTrackerStatus_t distance;

    if (!DistanceTracker_GetStatus(&distance))
    {
        return;
    }

    s_status.center_distance_mm =
        distance.center_signed_mm;

    s_status.traveled_distance_mm =
        distance.traveled_mm;

    TaskMenuUi_SetDistanceMm(
        distance.traveled_mm);
}

static void MainApp_LogStateIfChanged(void)
{
    if (s_last_logged_state ==
        s_status.state)
    {
        return;
    }

    s_last_logged_state =
        s_status.state;

    (void)BSP_Debug_Printf(
        "APP,STATE=%s,TASK=%u,NAME=%s,WARN=0x%08lX\r\n",
        MainApp_StateName(s_status.state),
        (unsigned int)s_status.selected_task,
        TaskMenuUi_TaskName(
            s_status.selected_task),
        (unsigned long)s_status.warning_mask);
}

static void MainApp_ReportPeriodic(
    uint32_t now_ms)
{
    AppTaskManagerStatus_t task_status;

    if (!MainApp_Elapsed(
            now_ms,
            s_last_debug_report_ms,
            APP_DEBUG_REPORT_PERIOD_MS))
    {
        return;
    }

    s_last_debug_report_ms = now_ms;

    if (AppTaskManager_GetStatus(
            &task_status))
    {
        (void)BSP_Debug_Printf(
            "APP,ST=%s,TASK=%u,TM=%s,"
            "ELAPSED=%lu,DIST=%lu,SDIST=%ld,"
            "OLED=%u,WARN=0x%08lX\r\n",
            MainApp_StateName(s_status.state),
            (unsigned int)s_status.selected_task,
            AppTaskManager_StateName(
                task_status.state),
            (unsigned long)s_status.elapsed_ms,
            (unsigned long)s_status.traveled_distance_mm,
            (long)s_status.center_distance_mm,
            BSP_Oled_IsOnline() ? 1U : 0U,
            (unsigned long)s_status.warning_mask);
    }
}

bool MainApp_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    s_key_ready = false;
    s_oled_ready = false;
    s_ui_ready = false;
    s_task_manager_ready = false;

    s_status.initialized = false;
    s_status.state = APP_STATE_BOOT;
    s_status.selected_task =
        TASK_MENU_TASK_2_LAP_STOP;
    s_status.state_timestamp_ms = now_ms;
    s_status.run_start_timestamp_ms = 0U;
    s_status.elapsed_ms = 0U;
    s_status.center_distance_mm = 0;
    s_status.traveled_distance_mm = 0U;
#if APP_TASK_PORT_PLACEHOLDER
    s_status.warning_mask =
        APP_WARNING_TASK_PLACEHOLDER;
#else
    s_status.warning_mask =
        APP_WARNING_NONE;
#endif
    s_status.fault_code = APP_FAULT_NONE;
    s_status.fault_detail = 0U;

    AppFault_Reset();

    if (!BSP_DebugUart_Init())
    {
        AppFault_Raise(
            APP_FAULT_DEBUG_UART_INIT,
            0U);
        return false;
    }

    if (!BSP_Oled_Init())
    {
        AppFault_Raise(
            APP_FAULT_OLED_INIT,
            0U);
        return false;
    }
    s_oled_ready = true;

    if (!TaskMenuUi_Init())
    {
        s_status.initialized = true;
        MainApp_EnterFault(
            APP_FAULT_MENU_INIT,
            0U);
        return true;
    }
    s_ui_ready = true;

    TaskMenuUi_SetWarningMask(
        s_status.warning_mask);
    TaskMenuUi_SetStatusText(
        "INITIALIZING");

    s_status.initialized = true;
    s_last_logged_state =
        APP_STATE_FAULT;
    s_last_debug_report_ms = now_ms;

    if (!BSP_Key_Init())
    {
        MainApp_EnterFault(
            APP_FAULT_KEY_INIT,
            0U);
        return true;
    }
    s_key_ready = true;

    if (!AppTaskManager_Init())
    {
        MainApp_EnterFault(
            APP_FAULT_TASK_MANAGER_INIT,
            0U);
        return true;
    }
    s_task_manager_ready = true;

    MainApp_SetState(
        APP_STATE_BOOT,
        now_ms);

    (void)BSP_Debug_Printf(
#if APP_TASK_PORT_PLACEHOLDER
        "APP,INIT=OK,ENTRY=MAIN_APP,"
        "TASK_PORT=SAFE_PLACEHOLDER\r\n");
#else
        "APP,INIT=OK,ENTRY=MAIN_APP,"
        "TASK_PORT=ACTIVE\r\n");
#endif

    return true;
}

void MainApp_Update(void)
{
    uint32_t now_ms;
    TaskMenuTask_t requested_task;
    AppTaskManagerStatus_t task_status;

    BSP_DebugUart_Process();

    if (!s_status.initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if (s_key_ready)
    {
        BSP_Key_Process();
    }

    if (s_ui_ready)
    {
        TaskMenuUi_Process();
    }

    if (s_task_manager_ready)
    {
        AppTaskManager_Update();
    }
    MainApp_UpdateDistance();

    switch (s_status.state)
    {
        case APP_STATE_BOOT:
            if (MainApp_Elapsed(
                    now_ms,
                    s_status.state_timestamp_ms,
                    APP_BOOT_DELAY_MS))
            {
                TaskMenuUi_SetStatusText(
                    "OLED CONNECT");
                MainApp_SetState(
                    APP_STATE_SELF_CHECK,
                    now_ms);
            }
            break;

        case APP_STATE_SELF_CHECK:
            if (BSP_Oled_IsOnline())
            {
                s_status.warning_mask &=
                    ~APP_WARNING_OLED_OFFLINE;

                TaskMenuUi_SetWarningMask(
                    s_status.warning_mask);

                MainApp_SetState(
                    APP_STATE_MENU,
                    now_ms);
            }
            else if (MainApp_Elapsed(
                         now_ms,
                         s_status.state_timestamp_ms,
                         APP_SELF_CHECK_TIMEOUT_MS))
            {
                /*
                 * OLED掉线不阻塞整个App。
                 * BSP_Oled仍会后台持续重连。
                 */
                s_status.warning_mask |=
                    APP_WARNING_OLED_OFFLINE;

                TaskMenuUi_SetWarningMask(
                    s_status.warning_mask);

                MainApp_SetState(
                    APP_STATE_MENU,
                    now_ms);
            }
            break;

        case APP_STATE_MENU:
            s_status.selected_task =
                TaskMenuUi_GetSelectedTask();

            if (TaskMenuUi_GetState() ==
                TASK_MENU_STATE_ARMED)
            {
                MainApp_SetState(
                    APP_STATE_ARMED,
                    now_ms);
            }
            break;

        case APP_STATE_ARMED:
            s_status.selected_task =
                TaskMenuUi_GetSelectedTask();

            if (TaskMenuUi_GetState() ==
                TASK_MENU_STATE_SELECT)
            {
                MainApp_SetState(
                    APP_STATE_MENU,
                    now_ms);
                break;
            }

            if (TaskMenuUi_TakeStartRequest(
                    &requested_task))
            {
                s_status.selected_task =
                    requested_task;
                s_status.run_start_timestamp_ms =
                    now_ms;
                s_status.elapsed_ms = 0U;

                if (!AppTaskManager_Start(
                        requested_task,
                        now_ms))
                {
                    MainApp_EnterFault(
                        APP_FAULT_TASK_START,
                        0U);
                    break;
                }

                MainApp_SetState(
                    APP_STATE_STARTING,
                    now_ms);
            }
            break;

        case APP_STATE_STARTING:
            MainApp_UpdateTaskStatus();

            if (TaskMenuUi_TakeStopRequest())
            {
                if (!AppTaskManager_RequestStop(
                        APP_TASK_STOP_USER))
                {
                    MainApp_EnterFault(
                        APP_FAULT_TASK_RUNTIME,
                        1U);
                    break;
                }

                MainApp_SetState(
                    APP_STATE_STOPPING,
                    now_ms);
            }
            else if (AppTaskManager_IsFaulted())
            {
                MainApp_EnterFault(
                    APP_FAULT_TASK_RUNTIME,
                    2U);
            }
            else if (AppTaskManager_IsRunning())
            {
                if (AppTaskManager_GetStatus(
                        &task_status) &&
                    (task_status.state ==
                     APP_TASK_MANAGER_RUNNING))
                {
                    MainApp_SetState(
                        APP_STATE_RUNNING,
                        now_ms);
                }
            }
            break;

        case APP_STATE_RUNNING:
            MainApp_UpdateTaskStatus();

            if (TaskMenuUi_TakeStopRequest())
            {
                if (!AppTaskManager_RequestStop(
                        APP_TASK_STOP_USER))
                {
                    MainApp_EnterFault(
                        APP_FAULT_TASK_RUNTIME,
                        3U);
                    break;
                }

                MainApp_SetState(
                    APP_STATE_STOPPING,
                    now_ms);
            }
            else if (AppTaskManager_IsFaulted())
            {
                MainApp_EnterFault(
                    APP_FAULT_TASK_RUNTIME,
                    4U);
            }
            else if (AppTaskManager_GetStatus(
                         &task_status) &&
                     (task_status.state ==
                      APP_TASK_MANAGER_STOPPING))
            {
                MainApp_SetState(
                    APP_STATE_STOPPING,
                    now_ms);
            }
            else if (AppTaskManager_IsFinished())
            {
                MainApp_SetState(
                    APP_STATE_FINISHED,
                    now_ms);
            }
            break;

        case APP_STATE_STOPPING:
            MainApp_UpdateTaskStatus();

            if (AppTaskManager_IsFaulted())
            {
                MainApp_EnterFault(
                    APP_FAULT_TASK_STOP_TIMEOUT,
                    0U);
            }
            else if (AppTaskManager_IsFinished())
            {
                MainApp_SetState(
                    APP_STATE_FINISHED,
                    now_ms);
            }
            break;

        case APP_STATE_FINISHED:
            MainApp_UpdateTaskStatus();

            if (TaskMenuUi_TakeMenuRequest())
            {
                AppTaskManager_Reset();
                s_status.elapsed_ms = 0U;
                MainApp_SetState(
                    APP_STATE_MENU,
                    now_ms);
            }
            break;

        case APP_STATE_FAULT:
        default:
            if (TaskMenuUi_TakeResetRequest())
            {
                AppFault_Clear();
                AppTaskManager_Reset();

                s_status.fault_code =
                    APP_FAULT_NONE;
                s_status.fault_detail = 0U;
                s_status.elapsed_ms = 0U;

                MainApp_SetState(
                    APP_STATE_MENU,
                    now_ms);
            }
            break;
    }

    if (BSP_Oled_IsOnline())
    {
        s_status.warning_mask &=
            ~APP_WARNING_OLED_OFFLINE;
    }
    else
    {
        s_status.warning_mask |=
            APP_WARNING_OLED_OFFLINE;
    }

    TaskMenuUi_SetWarningMask(
        s_status.warning_mask);

    MainApp_LogStateIfChanged();
    MainApp_ReportPeriodic(now_ms);

    BSP_Oled_Process();
}

void MainApp_Shutdown(void)
{
    AppTaskPort_ForceSafeStop();

    if (!s_status.initialized)
    {
        return;
    }

    s_status.initialized = false;
}

bool MainApp_GetStatus(
    MainAppStatus_t *status)
{
    AppFaultStatus_t fault;

    if ((!s_status.initialized) ||
        (status == NULL))
    {
        return false;
    }

    *status = s_status;

    if (AppFault_GetStatus(&fault))
    {
        status->fault_code = fault.code;
        status->fault_detail = fault.detail;
    }

    return true;
}
