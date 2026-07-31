#include "task_menu_ui.h"

#include "app_config.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "oled_fullscreen_timer.h"
#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <string.h>

#define TASK_MENU_STATUS_TEXT_SIZE      22U

static bool s_initialized = false;
static bool s_display_dirty = false;
static bool s_fullscreen_timer_enabled = false;

static TaskMenuTask_t s_selected_task =
    TASK_MENU_TASK_2_LAP_STOP;

static TaskMenuState_t s_state =
    TASK_MENU_STATE_BOOT;

static bool s_start_request = false;
static bool s_stop_request = false;
static bool s_menu_request = false;
static bool s_reset_request = false;

static uint32_t s_elapsed_ms = 0U;
static uint32_t s_distance_mm = 0U;
static uint32_t s_warning_mask = 0U;
static uint32_t s_fault_code = 0U;
static uint32_t s_fault_detail = 0U;

static uint32_t s_last_render_ms = 0U;
static uint32_t s_last_render_decisecond = 0U;

static uint32_t s_selection_change_count = 0U;
static uint32_t s_start_request_count = 0U;
static uint32_t s_stop_request_count = 0U;
static uint32_t s_menu_request_count = 0U;
static uint32_t s_reset_request_count = 0U;

static char s_status_text[TASK_MENU_STATUS_TEXT_SIZE];
static char s_fault_name[TASK_MENU_STATUS_TEXT_SIZE];

static void TaskMenu_ClearRequests(void)
{
    s_start_request = false;
    s_stop_request = false;
    s_menu_request = false;
    s_reset_request = false;
}

static void TaskMenu_CopyText(
    char *destination,
    const char *source)
{
    uint8_t index = 0U;

    if (destination == NULL)
    {
        return;
    }

    if (source != NULL)
    {
        while ((source[index] != '\0') &&
               (index <
                (TASK_MENU_STATUS_TEXT_SIZE - 1U)))
        {
            destination[index] = source[index];
            index++;
        }
    }

    destination[index] = '\0';
}

static TaskMenuTask_t TaskMenu_NextTask(
    TaskMenuTask_t task)
{
    if (task >= TASK_MENU_TASK_6_LAP_TARGET)
    {
        return TASK_MENU_TASK_2_LAP_STOP;
    }

    return (TaskMenuTask_t)((uint8_t)task + 1U);
}

const char *TaskMenuUi_TaskName(
    TaskMenuTask_t task)
{
    switch (task)
    {
        case TASK_MENU_TASK_2_LAP_STOP:
            return "2 LAP STOP";

        case TASK_MENU_TASK_3_BALL_SEQUENCE:
            return "3 BALL SEQ";

        case TASK_MENU_TASK_4_AB_HOLD:
            return "4 A-B HOLD";

        case TASK_MENU_TASK_5_LAP_HOLD:
            return "5 LAP HOLD";

        case TASK_MENU_TASK_6_LAP_TARGET:
            return "6 LAP TARGET";

        default:
            return "?";
    }
}

const char *TaskMenuUi_StateName(
    TaskMenuState_t state)
{
    switch (state)
    {
        case TASK_MENU_STATE_BOOT:
            return "BOOT";

        case TASK_MENU_STATE_SELF_CHECK:
            return "CHECK";

        case TASK_MENU_STATE_SELECT:
            return "SELECT";

        case TASK_MENU_STATE_ARMED:
            return "ARMED";

        case TASK_MENU_STATE_STARTING:
            return "STARTING";

        case TASK_MENU_STATE_RUNNING:
            return "RUNNING";

        case TASK_MENU_STATE_STOPPING:
            return "STOPPING";

        case TASK_MENU_STATE_FINISHED:
            return "FINISHED";

        case TASK_MENU_STATE_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

static void TaskMenu_DrawTaskLine(
    uint8_t page,
    TaskMenuTask_t task)
{
    BSP_Oled_DrawString(
        0U,
        page,
        (task == s_selected_task) ? ">" : " ");

    BSP_Oled_DrawString(
        12U,
        page,
        TaskMenuUi_TaskName(task));
}

static void TaskMenu_DrawElapsed(
    uint8_t page)
{
    uint32_t seconds =
        s_elapsed_ms / 1000U;
    uint32_t decisecond =
        (s_elapsed_ms / 100U) % 10U;

    BSP_Oled_DrawString(0U, page, "T:");
    BSP_Oled_DrawU32(12U, page, seconds);
    BSP_Oled_DrawString(72U, page, ".");
    BSP_Oled_DrawU32(78U, page, decisecond);
    BSP_Oled_DrawString(90U, page, "S");
}

static void TaskMenu_DrawDistance(uint8_t page)
{
    BSP_Oled_DrawString(0U, page, "D:");
    BSP_Oled_DrawU32(12U, page, s_distance_mm);
    BSP_Oled_DrawString(84U, page, "MM");
}

static void TaskMenu_RenderBoot(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 1U, "2026H SYSTEM");
    BSP_Oled_DrawString(0U, 3U, "MAIN APP V1");
    BSP_Oled_DrawString(0U, 5U, "BOOTING...");
}

static void TaskMenu_RenderSelfCheck(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "SELF CHECK");
    BSP_Oled_DrawString(0U, 2U, "KEY : OK");
    BSP_Oled_DrawString(
        0U,
        3U,
        BSP_Oled_IsOnline() ?
            "OLED: OK" :
            "OLED: RETRY");
    BSP_Oled_DrawString(0U, 5U, s_status_text);

    if ((s_warning_mask &
         APP_WARNING_TASK_PLACEHOLDER) != 0U)
    {
        BSP_Oled_DrawString(
            0U,
            7U,
            "TASK: SKELETON");
    }
}

static void TaskMenu_RenderSelect(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "2026H TASK SELECT");

    TaskMenu_DrawTaskLine(
        1U,
        TASK_MENU_TASK_2_LAP_STOP);
    TaskMenu_DrawTaskLine(
        2U,
        TASK_MENU_TASK_3_BALL_SEQUENCE);
    TaskMenu_DrawTaskLine(
        3U,
        TASK_MENU_TASK_4_AB_HOLD);
    TaskMenu_DrawTaskLine(
        4U,
        TASK_MENU_TASK_5_LAP_HOLD);
    TaskMenu_DrawTaskLine(
        5U,
        TASK_MENU_TASK_6_LAP_TARGET);

    BSP_Oled_DrawString(
        0U,
        7U,
        "UP:NEXT K0:OK");
}

static void TaskMenu_RenderArmed(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK ARMED");
    BSP_Oled_DrawString(
        0U,
        2U,
        TaskMenuUi_TaskName(s_selected_task));
    BSP_Oled_DrawString(0U, 4U, "K0:START");
    BSP_Oled_DrawString(0U, 6U, "UP:BACK");
}

static void TaskMenu_RenderStarting(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK STARTING");
    BSP_Oled_DrawString(
        0U,
        2U,
        TaskMenuUi_TaskName(s_selected_task));
    TaskMenu_DrawElapsed(4U);
    BSP_Oled_DrawString(0U, 6U, s_status_text);
    BSP_Oled_DrawString(0U, 7U, "K0:STOP");
}

static void TaskMenu_RenderRunning(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK RUNNING");
    BSP_Oled_DrawString(
        0U,
        1U,
        TaskMenuUi_TaskName(s_selected_task));
    TaskMenu_DrawElapsed(3U);
    TaskMenu_DrawDistance(4U);
    BSP_Oled_DrawString(0U, 5U, s_status_text);

    if ((s_warning_mask &
         APP_WARNING_TASK_PLACEHOLDER) != 0U)
    {
        BSP_Oled_DrawString(
            0U,
            6U,
            "NO MOTION OUTPUT");
    }

    BSP_Oled_DrawString(0U, 7U, "K0:STOP");
}

static void TaskMenu_RenderStopping(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK STOPPING");
    BSP_Oled_DrawString(
        0U,
        2U,
        TaskMenuUi_TaskName(s_selected_task));
    TaskMenu_DrawElapsed(4U);
    TaskMenu_DrawDistance(5U);
    BSP_Oled_DrawString(0U, 6U, s_status_text);
}

static void TaskMenu_RenderFinished(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK FINISHED");
    BSP_Oled_DrawString(
        0U,
        2U,
        TaskMenuUi_TaskName(s_selected_task));
    TaskMenu_DrawElapsed(4U);
    TaskMenu_DrawDistance(5U);
    BSP_Oled_DrawString(0U, 7U, "K0:MENU");
}

static void TaskMenu_RenderFault(void)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "FAULT");
    BSP_Oled_DrawString(0U, 2U, s_fault_name);
    BSP_Oled_DrawString(0U, 4U, "CODE:");
    BSP_Oled_DrawU32(36U, 4U, s_fault_code);
    BSP_Oled_DrawString(0U, 6U, "DETAIL:");
    BSP_Oled_DrawU32(42U, 6U, s_fault_detail);
    BSP_Oled_DrawString(0U, 7U, "K0:RESET");
}

static void TaskMenu_RenderCurrent(void)
{
    if (s_fullscreen_timer_enabled &&
        ((s_state == TASK_MENU_STATE_STARTING) ||
         (s_state == TASK_MENU_STATE_RUNNING) ||
         (s_state == TASK_MENU_STATE_STOPPING) ||
         (s_state == TASK_MENU_STATE_FINISHED)))
    {
        OledFullscreenTimer_ShowElapsedMs(
            s_elapsed_ms);
        s_display_dirty = false;
        s_last_render_ms = HAL_GetTick();
        s_last_render_decisecond =
            s_elapsed_ms / 100U;
        return;
    }

    switch (s_state)
    {
        case TASK_MENU_STATE_BOOT:
            TaskMenu_RenderBoot();
            break;

        case TASK_MENU_STATE_SELF_CHECK:
            TaskMenu_RenderSelfCheck();
            break;

        case TASK_MENU_STATE_SELECT:
            TaskMenu_RenderSelect();
            break;

        case TASK_MENU_STATE_ARMED:
            TaskMenu_RenderArmed();
            break;

        case TASK_MENU_STATE_STARTING:
            TaskMenu_RenderStarting();
            break;

        case TASK_MENU_STATE_RUNNING:
            TaskMenu_RenderRunning();
            break;

        case TASK_MENU_STATE_STOPPING:
            TaskMenu_RenderStopping();
            break;

        case TASK_MENU_STATE_FINISHED:
            TaskMenu_RenderFinished();
            break;

        case TASK_MENU_STATE_FAULT:
        default:
            TaskMenu_RenderFault();
            break;
    }

    s_display_dirty = false;
    s_last_render_ms = HAL_GetTick();
    s_last_render_decisecond =
        s_elapsed_ms / 100U;
}

bool TaskMenuUi_Init(void)
{
    s_selected_task =
        TASK_MENU_TASK_2_LAP_STOP;
    s_state = TASK_MENU_STATE_BOOT;
    s_fullscreen_timer_enabled = false;

    TaskMenu_ClearRequests();

    s_elapsed_ms = 0U;
    s_distance_mm = 0U;
    s_warning_mask = 0U;
    s_fault_code = 0U;
    s_fault_detail = 0U;

    s_selection_change_count = 0U;
    s_start_request_count = 0U;
    s_stop_request_count = 0U;
    s_menu_request_count = 0U;
    s_reset_request_count = 0U;

    TaskMenu_CopyText(
        s_status_text,
        "SAFE SKELETON");
    TaskMenu_CopyText(
        s_fault_name,
        "NONE");

    s_initialized = true;
    s_display_dirty = true;
    TaskMenu_RenderCurrent();

    return true;
}

void TaskMenuUi_Process(void)
{
    uint32_t now_ms;
    uint32_t current_decisecond;

    if (!s_initialized)
    {
        return;
    }

    switch (s_state)
    {
        case TASK_MENU_STATE_SELECT:
            if (BSP_Key_TakePress(
                    BSP_KEY_SELECT))
            {
                (void)BSP_Key_TakePress(
                    BSP_KEY_CONFIRM);
                s_selected_task =
                    TaskMenu_NextTask(
                        s_selected_task);
                s_selection_change_count++;
                s_display_dirty = true;
            }
            else if (BSP_Key_TakePress(
                    BSP_KEY_CONFIRM))
            {
                TaskMenu_ClearRequests();
                s_state = TASK_MENU_STATE_ARMED;
                s_display_dirty = true;
            }
            break;

        case TASK_MENU_STATE_ARMED:
            if (BSP_Key_TakePress(
                    BSP_KEY_SELECT))
            {
                (void)BSP_Key_TakePress(
                    BSP_KEY_CONFIRM);
                TaskMenu_ClearRequests();
                s_state = TASK_MENU_STATE_SELECT;
                s_display_dirty = true;
            }
            else if (BSP_Key_TakePress(
                    BSP_KEY_CONFIRM))
            {
                s_start_request = true;
                s_start_request_count++;
            }
            break;

        case TASK_MENU_STATE_STARTING:
        case TASK_MENU_STATE_RUNNING:
            (void)BSP_Key_TakePress(
                BSP_KEY_SELECT);

            if (BSP_Key_TakePress(
                    BSP_KEY_CONFIRM))
            {
                s_stop_request = true;
                s_stop_request_count++;
            }
            break;

        case TASK_MENU_STATE_FINISHED:
            (void)BSP_Key_TakePress(
                BSP_KEY_SELECT);

            if (BSP_Key_TakePress(
                    BSP_KEY_CONFIRM))
            {
                s_menu_request = true;
                s_menu_request_count++;
            }
            break;

        case TASK_MENU_STATE_FAULT:
            (void)BSP_Key_TakePress(
                BSP_KEY_SELECT);

            if (BSP_Key_TakePress(
                    BSP_KEY_CONFIRM))
            {
                s_reset_request = true;
                s_reset_request_count++;
            }
            break;

        case TASK_MENU_STATE_BOOT:
        case TASK_MENU_STATE_SELF_CHECK:
        case TASK_MENU_STATE_STOPPING:
        default:
            (void)BSP_Key_TakePress(
                BSP_KEY_SELECT);
            (void)BSP_Key_TakePress(
                BSP_KEY_CONFIRM);
            break;
    }

    now_ms = HAL_GetTick();
    current_decisecond =
        s_elapsed_ms / 100U;

    if (((s_state == TASK_MENU_STATE_STARTING) ||
         (s_state == TASK_MENU_STATE_RUNNING) ||
         (s_state == TASK_MENU_STATE_STOPPING)) &&
        (current_decisecond !=
         s_last_render_decisecond) &&
        ((uint32_t)(now_ms - s_last_render_ms) >=
         APP_UI_RUNTIME_REFRESH_MS))
    {
        s_display_dirty = true;
    }

    if (s_display_dirty)
    {
        TaskMenu_RenderCurrent();
    }
}

void TaskMenuUi_SetState(
    TaskMenuState_t state)
{
    if ((!s_initialized) ||
        (s_state == state))
    {
        return;
    }

    TaskMenu_ClearRequests();
    s_state = state;
    s_display_dirty = true;
}

void TaskMenuUi_SetElapsedMs(
    uint32_t elapsed_ms)
{
    s_elapsed_ms = elapsed_ms;
}

void TaskMenuUi_SetDistanceMm(
    uint32_t distance_mm)
{
    if (s_distance_mm != distance_mm)
    {
        s_distance_mm = distance_mm;

        if ((s_state == TASK_MENU_STATE_STARTING) ||
            (s_state == TASK_MENU_STATE_RUNNING) ||
            (s_state == TASK_MENU_STATE_STOPPING) ||
            (s_state == TASK_MENU_STATE_FINISHED))
        {
            s_display_dirty = true;
        }
    }
}

void TaskMenuUi_SetWarningMask(
    uint32_t warning_mask)
{
    if (s_warning_mask != warning_mask)
    {
        s_warning_mask = warning_mask;
        s_display_dirty = true;
    }
}

void TaskMenuUi_SetStatusText(
    const char *text)
{
    char previous[TASK_MENU_STATUS_TEXT_SIZE];

    memcpy(
        previous,
        s_status_text,
        sizeof(previous));

    TaskMenu_CopyText(
        s_status_text,
        text);

    if (memcmp(
            previous,
            s_status_text,
            sizeof(previous)) != 0)
    {
        s_display_dirty = true;
    }
}

void TaskMenuUi_SetFullscreenTimerEnabled(bool enabled)
{
    if ((!s_initialized) ||
        (s_fullscreen_timer_enabled == enabled))
    {
        return;
    }

    s_fullscreen_timer_enabled = enabled;
    OledFullscreenTimer_Reset();
    s_display_dirty = true;
}

void TaskMenuUi_SetFault(
    uint32_t fault_code,
    uint32_t fault_detail,
    const char *fault_name)
{
    TaskMenu_ClearRequests();
    s_fault_code = fault_code;
    s_fault_detail = fault_detail;
    TaskMenu_CopyText(
        s_fault_name,
        fault_name);
    s_state = TASK_MENU_STATE_FAULT;
    s_display_dirty = true;
}

bool TaskMenuUi_TakeStartRequest(
    TaskMenuTask_t *task)
{
    if ((!s_initialized) ||
        (!s_start_request))
    {
        return false;
    }

    s_start_request = false;

    if (task != NULL)
    {
        *task = s_selected_task;
    }

    return true;
}

bool TaskMenuUi_TakeStopRequest(void)
{
    if ((!s_initialized) ||
        (!s_stop_request))
    {
        return false;
    }

    s_stop_request = false;
    return true;
}

bool TaskMenuUi_TakeMenuRequest(void)
{
    if ((!s_initialized) ||
        (!s_menu_request))
    {
        return false;
    }

    s_menu_request = false;
    return true;
}

bool TaskMenuUi_TakeResetRequest(void)
{
    if ((!s_initialized) ||
        (!s_reset_request))
    {
        return false;
    }

    s_reset_request = false;
    return true;
}

void TaskMenuUi_SetFinished(void)
{
    TaskMenuUi_SetState(
        TASK_MENU_STATE_FINISHED);
}

TaskMenuTask_t TaskMenuUi_GetSelectedTask(void)
{
    return s_selected_task;
}

TaskMenuState_t TaskMenuUi_GetState(void)
{
    return s_state;
}

bool TaskMenuUi_GetStatus(
    TaskMenuStatus_t *status)
{
    if ((!s_initialized) ||
        (status == NULL))
    {
        return false;
    }

    status->initialized = s_initialized;
    status->selected_task = s_selected_task;
    status->state = s_state;
    status->elapsed_ms = s_elapsed_ms;
    status->distance_mm = s_distance_mm;
    status->warning_mask = s_warning_mask;
    status->fault_code = s_fault_code;
    status->fault_detail = s_fault_detail;

    status->selection_change_count =
        s_selection_change_count;
    status->start_request_count =
        s_start_request_count;
    status->stop_request_count =
        s_stop_request_count;
    status->menu_request_count =
        s_menu_request_count;
    status->reset_request_count =
        s_reset_request_count;

    return true;
}
