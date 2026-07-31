#include "task_menu_ui.h"

#include "bsp_key.h"
#include "bsp_oled.h"

#include <stddef.h>

static bool s_initialized = false;

static TaskMenuTask_t s_selected_task =
    TASK_MENU_TASK_2_LAP_STOP;

static TaskMenuState_t s_state =
    TASK_MENU_STATE_SELECT;

static bool s_start_request = false;
static bool s_stop_request = false;

static uint32_t s_selection_change_count = 0U;
static uint32_t s_start_request_count = 0U;
static uint32_t s_stop_request_count = 0U;

static TaskMenuTask_t TaskMenu_NextTask(
    TaskMenuTask_t task)
{
    if (task >= TASK_MENU_TASK_6_LAP_TARGET)
    {
        return TASK_MENU_TASK_2_LAP_STOP;
    }

    return (TaskMenuTask_t)((uint8_t)task + 1U);
}

const char *TaskMenuUi_TaskName(TaskMenuTask_t task)
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

const char *TaskMenuUi_StateName(TaskMenuState_t state)
{
    switch (state)
    {
        case TASK_MENU_STATE_SELECT:
            return "SELECT";

        case TASK_MENU_STATE_ARMED:
            return "ARMED";

        case TASK_MENU_STATE_RUNNING:
            return "RUN";

        case TASK_MENU_STATE_FINISHED:
            return "DONE";

        default:
            return "UNKNOWN";
    }
}

static void TaskMenu_DrawTaskLine(
    uint8_t page,
    TaskMenuTask_t task)
{
    if (task == s_selected_task)
    {
        BSP_Oled_DrawString(
            0U,
            page,
            ">");
    }
    else
    {
        BSP_Oled_DrawString(
            0U,
            page,
            " ");
    }

    BSP_Oled_DrawString(
        12U,
        page,
        TaskMenuUi_TaskName(task));
}

static void TaskMenu_RenderSelect(void)
{
    BSP_Oled_Clear();

    BSP_Oled_DrawString(
        0U,
        0U,
        "2026H TASK SELECT");

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

    BSP_Oled_DrawString(
        0U,
        0U,
        "TASK ARMED");

    BSP_Oled_DrawString(
        0U,
        2U,
        TaskMenuUi_TaskName(s_selected_task));

    BSP_Oled_DrawString(
        0U,
        4U,
        "K0:START");

    BSP_Oled_DrawString(
        0U,
        6U,
        "UP:BACK");
}

static void TaskMenu_RenderRunning(void)
{
    BSP_Oled_Clear();

    BSP_Oled_DrawString(
        0U,
        0U,
        "TASK RUN REQUEST");

    BSP_Oled_DrawString(
        0U,
        2U,
        TaskMenuUi_TaskName(s_selected_task));

    BSP_Oled_DrawString(
        0U,
        4U,
        "NO TASK LOGIC");

    BSP_Oled_DrawString(
        0U,
        6U,
        "K0:STOP");
}

static void TaskMenu_RenderFinished(void)
{
    BSP_Oled_Clear();

    BSP_Oled_DrawString(
        0U,
        0U,
        "TASK STOPPED");

    BSP_Oled_DrawString(
        0U,
        2U,
        TaskMenuUi_TaskName(s_selected_task));

    BSP_Oled_DrawString(
        0U,
        5U,
        "K0:MENU");
}

static void TaskMenu_RenderCurrent(void)
{
    switch (s_state)
    {
        case TASK_MENU_STATE_SELECT:
            TaskMenu_RenderSelect();
            break;

        case TASK_MENU_STATE_ARMED:
            TaskMenu_RenderArmed();
            break;

        case TASK_MENU_STATE_RUNNING:
            TaskMenu_RenderRunning();
            break;

        case TASK_MENU_STATE_FINISHED:
        default:
            TaskMenu_RenderFinished();
            break;
    }
}

bool TaskMenuUi_Init(void)
{
    s_selected_task =
        TASK_MENU_TASK_2_LAP_STOP;
    s_state = TASK_MENU_STATE_SELECT;

    s_start_request = false;
    s_stop_request = false;

    s_selection_change_count = 0U;
    s_start_request_count = 0U;
    s_stop_request_count = 0U;

    s_initialized = true;
    TaskMenu_RenderCurrent();

    return true;
}

void TaskMenuUi_Process(void)
{
    bool redraw = false;

    if (!s_initialized)
    {
        return;
    }

    switch (s_state)
    {
        case TASK_MENU_STATE_SELECT:
            if (BSP_Key_TakePress(BSP_KEY_SELECT))
            {
                s_selected_task =
                    TaskMenu_NextTask(s_selected_task);
                s_selection_change_count++;
                redraw = true;
            }

            if (BSP_Key_TakePress(BSP_KEY_CONFIRM))
            {
                s_state = TASK_MENU_STATE_ARMED;
                redraw = true;
            }
            break;

        case TASK_MENU_STATE_ARMED:
            if (BSP_Key_TakePress(BSP_KEY_SELECT))
            {
                s_state = TASK_MENU_STATE_SELECT;
                redraw = true;
            }

            if (BSP_Key_TakePress(BSP_KEY_CONFIRM))
            {
                s_start_request = true;
                s_start_request_count++;
                s_state = TASK_MENU_STATE_RUNNING;
                redraw = true;
            }
            break;

        case TASK_MENU_STATE_RUNNING:
            if (BSP_Key_TakePress(BSP_KEY_CONFIRM))
            {
                s_stop_request = true;
                s_stop_request_count++;
                s_state = TASK_MENU_STATE_FINISHED;
                redraw = true;
            }
            else
            {
                /* 运行中忽略选择键。 */
                (void)BSP_Key_TakePress(BSP_KEY_SELECT);
            }
            break;

        case TASK_MENU_STATE_FINISHED:
        default:
            (void)BSP_Key_TakePress(BSP_KEY_SELECT);

            if (BSP_Key_TakePress(BSP_KEY_CONFIRM))
            {
                s_state = TASK_MENU_STATE_SELECT;
                redraw = true;
            }
            break;
    }

    if (redraw)
    {
        TaskMenu_RenderCurrent();
    }
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

void TaskMenuUi_SetFinished(void)
{
    if (!s_initialized)
    {
        return;
    }

    s_state = TASK_MENU_STATE_FINISHED;
    TaskMenu_RenderCurrent();
}

TaskMenuTask_t TaskMenuUi_GetSelectedTask(void)
{
    return s_selected_task;
}

TaskMenuState_t TaskMenuUi_GetState(void)
{
    return s_state;
}

bool TaskMenuUi_GetStatus(TaskMenuStatus_t *status)
{
    if ((!s_initialized) || (status == NULL))
    {
        return false;
    }

    status->initialized = s_initialized;
    status->selected_task = s_selected_task;
    status->state = s_state;

    status->selection_change_count =
        s_selection_change_count;
    status->start_request_count =
        s_start_request_count;
    status->stop_request_count =
        s_stop_request_count;

    return true;
}
