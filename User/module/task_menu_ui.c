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
static bool s_finished_has_result = false;
static bool s_finished_passed = false;
static uint32_t s_finished_elapsed_ms = 0U;
static uint32_t s_running_elapsed_ms = 0U;
static bool s_running_timer_started = false;
static bool s_last_rendered_timer_started = false;
static uint32_t s_last_rendered_tenth = UINT32_MAX;

/* bf89793-compatible 28x46 seven-segment timer: SS.T seconds. */
#define TASK_TIMER_DIGIT_W       28U
#define TASK_TIMER_DIGIT_H       46U
#define TASK_TIMER_START_Y        9U
#define TASK_TIMER_DP_W           8U
#define TASK_TIMER_DP_H           8U
#define TASK_TIMER_GAP_DIGIT      4U
#define TASK_TIMER_GAP_DP         2U
#define TASK_TIMER_TOTAL_W \
    ((TASK_TIMER_DIGIT_W * 3U) + (TASK_TIMER_GAP_DIGIT * 2U) + \
     TASK_TIMER_DP_W + (TASK_TIMER_GAP_DP * 2U))
#define TASK_TIMER_START_X \
    ((BSP_OLED_WIDTH - TASK_TIMER_TOTAL_W) / 2U)

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t w;
    uint8_t h;
} TaskTimerSegment_t;

static const TaskTimerSegment_t s_seg_a = {4U,  0U,  20U, 6U};
static const TaskTimerSegment_t s_seg_b = {24U, 5U,  4U,  17U};
static const TaskTimerSegment_t s_seg_c = {24U, 24U, 4U,  17U};
static const TaskTimerSegment_t s_seg_d = {4U,  40U, 20U, 6U};
static const TaskTimerSegment_t s_seg_e = {0U,  24U, 4U,  17U};
static const TaskTimerSegment_t s_seg_f = {0U,  5U,  4U,  17U};
static const TaskTimerSegment_t s_seg_g = {4U,  22U, 20U, 6U};

static const uint8_t s_seg_map[10] =
{
    0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
    0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
};

static const TaskTimerSegment_t * const s_seg_table[7] =
{
    &s_seg_a, &s_seg_b, &s_seg_c, &s_seg_d,
    &s_seg_e, &s_seg_f, &s_seg_g
};

static void TaskMenu_DrawLargeDigit(
    uint8_t digit,
    uint8_t dx,
    uint8_t dy)
{
    uint8_t mask;
    uint8_t i;
    const TaskTimerSegment_t *seg;

    if (digit > 9U)
    {
        return;
    }

    mask = s_seg_map[digit];
    for (i = 0U; i < 7U; i++)
    {
        if ((mask & (uint8_t)(1U << i)) != 0U)
        {
            seg = s_seg_table[i];
            BSP_Oled_FillRect(
                (uint8_t)(dx + seg->x),
                (uint8_t)(dy + seg->y),
                seg->w,
                seg->h,
                true);
        }
    }
}

static void TaskMenu_DrawLargeTime(uint32_t elapsed_ms)
{
    uint32_t tenths;
    uint32_t seconds;
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t x;

    tenths = elapsed_ms / 100U;
    if (tenths > 999U)
    {
        tenths = 999U;
    }

    seconds = tenths / 10U;
    d0 = (uint8_t)((seconds / 10U) % 10U);
    d1 = (uint8_t)(seconds % 10U);
    d2 = (uint8_t)(tenths % 10U);

    x = TASK_TIMER_START_X;
    TaskMenu_DrawLargeDigit(d0, x, TASK_TIMER_START_Y);
    x = (uint8_t)(x + TASK_TIMER_DIGIT_W + TASK_TIMER_GAP_DIGIT);
    TaskMenu_DrawLargeDigit(d1, x, TASK_TIMER_START_Y);
    x = (uint8_t)(x + TASK_TIMER_DIGIT_W + TASK_TIMER_GAP_DP);
    BSP_Oled_FillRect(
        x,
        (uint8_t)(TASK_TIMER_START_Y + TASK_TIMER_DIGIT_H - TASK_TIMER_DP_H),
        TASK_TIMER_DP_W,
        TASK_TIMER_DP_H,
        true);
    x = (uint8_t)(x + TASK_TIMER_DP_W + TASK_TIMER_GAP_DP);
    TaskMenu_DrawLargeDigit(d2, x, TASK_TIMER_START_Y);
}

static void TaskMenu_RenderTask3Timer(
    uint32_t elapsed_ms,
    const char *state_text,
    const char *bottom_text)
{
    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK3");
    BSP_Oled_DrawString(94U, 0U, state_text);
    TaskMenu_DrawLargeTime(elapsed_ms);
    BSP_Oled_DrawString(30U, 7U, bottom_text);
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
    if (s_selected_task == TASK_MENU_TASK_3_BALL_SEQUENCE)
    {
        TaskMenu_RenderTask3Timer(
            s_running_elapsed_ms,
            s_running_timer_started ? "RUN" : "RDY",
            "K0 STOP");
        return;
    }

    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK RUN REQUEST");
    BSP_Oled_DrawString(0U, 2U, TaskMenuUi_TaskName(s_selected_task));
    BSP_Oled_DrawString(0U, 4U, "NOT IMPLEMENTED");
    BSP_Oled_DrawString(0U, 6U, "K0:STOP");
}

static void TaskMenu_RenderFinished(void)
{
    if ((s_selected_task == TASK_MENU_TASK_3_BALL_SEQUENCE) &&
        s_finished_has_result)
    {
        TaskMenu_RenderTask3Timer(
            s_finished_elapsed_ms,
            s_finished_passed ? "PASS" : "FAIL",
            s_finished_passed ? "HOLD K0" : "K0 MENU");
        return;
    }

    BSP_Oled_Clear();
    BSP_Oled_DrawString(0U, 0U, "TASK FINISHED");
    BSP_Oled_DrawString(0U, 2U, TaskMenuUi_TaskName(s_selected_task));
    BSP_Oled_DrawString(0U, 7U, "K0:MENU");
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
    s_finished_has_result = false;
    s_finished_passed = false;
    s_finished_elapsed_ms = 0U;
    s_running_elapsed_ms = 0U;
    s_running_timer_started = false;
    s_last_rendered_timer_started = false;
    s_last_rendered_tenth = UINT32_MAX;

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
                s_finished_has_result = false;
                s_finished_passed = false;
                s_finished_elapsed_ms = 0U;
                s_running_elapsed_ms = 0U;
                s_running_timer_started = false;
                s_last_rendered_timer_started = false;
                s_last_rendered_tenth = UINT32_MAX;
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
                /* Stop the final hold before returning to the menu. */
                s_stop_request = true;
                s_stop_request_count++;
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

    s_finished_has_result = false;
    s_state = TASK_MENU_STATE_FINISHED;
    TaskMenu_RenderCurrent();
}

void TaskMenuUi_SetRunningElapsedMs(
    uint32_t elapsed_ms,
    bool timer_running)
{
    uint32_t tenth;

    if ((!s_initialized) ||
        (s_state != TASK_MENU_STATE_RUNNING) ||
        (s_selected_task != TASK_MENU_TASK_3_BALL_SEQUENCE))
    {
        return;
    }

    s_running_elapsed_ms = elapsed_ms;
    s_running_timer_started = timer_running;
    tenth = elapsed_ms / 100U;

    if ((tenth != s_last_rendered_tenth) ||
        (timer_running != s_last_rendered_timer_started))
    {
        s_last_rendered_tenth = tenth;
        s_last_rendered_timer_started = timer_running;
        TaskMenu_RenderRunning();
    }
}

void TaskMenuUi_SetFinishedResult(
    bool passed,
    uint32_t elapsed_ms)
{
    if (!s_initialized)
    {
        return;
    }

    s_finished_has_result = true;
    s_finished_passed = passed;
    s_finished_elapsed_ms = elapsed_ms;
    s_running_elapsed_ms = elapsed_ms;
    s_running_timer_started = false;
    s_last_rendered_timer_started = false;
    s_last_rendered_tenth = elapsed_ms / 100U;
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
