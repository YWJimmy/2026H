#include "test_oled_key.h"

#include "bsp_debug_uart.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "task_menu_ui.h"

#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_OLED_KEY_REPORT_MS        1000U

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;
static uint32_t s_run_start_ms = 0U;

static TaskMenuTask_t s_last_task =
    TASK_MENU_TASK_2_LAP_STOP;

static TaskMenuState_t s_last_state =
    TASK_MENU_STATE_BOOT;

static bool s_last_oled_online = false;

static void Test_OledKey_ReportChange(void)
{
    TaskMenuTask_t task =
        TaskMenuUi_GetSelectedTask();

    TaskMenuState_t state =
        TaskMenuUi_GetState();

    bool online =
        BSP_Oled_IsOnline();

    if ((task != s_last_task) ||
        (state != s_last_state) ||
        (online != s_last_oled_online))
    {
        (void)BSP_Debug_Printf(
            "UI,TASK=%u,NAME=%s,STATE=%s,OLED=%u\r\n",
            (unsigned int)task,
            TaskMenuUi_TaskName(task),
            TaskMenuUi_StateName(state),
            online ? 1U : 0U);

        s_last_task = task;
        s_last_state = state;
        s_last_oled_online = online;
    }
}

bool Test_OledKey_Init(void)
{
    s_initialized = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_Key_Init())
    {
        return false;
    }

    if (!BSP_Oled_Init())
    {
        return false;
    }

    if (!TaskMenuUi_Init())
    {
        return false;
    }

    TaskMenuUi_SetState(
        TASK_MENU_STATE_SELECT);
    TaskMenuUi_SetStatusText(
        "UI TEST");

    s_last_task =
        TaskMenuUi_GetSelectedTask();
    s_last_state =
        TaskMenuUi_GetState();
    s_last_oled_online = false;
    s_last_report_ms = HAL_GetTick();
    s_run_start_ms = 0U;

    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,OLED_KEY,START,V2_STATE_UI\r\n");

    return true;
}

void Test_OledKey_Update(void)
{
    TaskMenuTask_t start_task;
    BspOledStatus_t oled;
    BspKeyStatus_t key;
    uint32_t now_ms;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    BSP_Key_Process();
    TaskMenuUi_Process();

    if (TaskMenuUi_TakeStartRequest(
            &start_task))
    {
        s_run_start_ms = now_ms;
        TaskMenuUi_SetStatusText(
            "UI TEST RUN");
        TaskMenuUi_SetState(
            TASK_MENU_STATE_RUNNING);

        (void)BSP_Debug_Printf(
            "UI,EVENT=START_REQUEST,TASK=%u\r\n",
            (unsigned int)start_task);
    }

    if (TaskMenuUi_GetState() ==
        TASK_MENU_STATE_RUNNING)
    {
        TaskMenuUi_SetElapsedMs(
            (uint32_t)(now_ms -
                       s_run_start_ms));
    }

    if (TaskMenuUi_TakeStopRequest())
    {
        TaskMenuUi_SetFinished();

        (void)BSP_Debug_Printf(
            "UI,EVENT=STOP_REQUEST\r\n");
    }

    if (TaskMenuUi_TakeMenuRequest())
    {
        TaskMenuUi_SetElapsedMs(0U);
        TaskMenuUi_SetState(
            TASK_MENU_STATE_SELECT);
    }

    Test_OledKey_ReportChange();
    BSP_Oled_Process();

    if ((uint32_t)(now_ms -
                   s_last_report_ms) >=
        TEST_OLED_KEY_REPORT_MS)
    {
        s_last_report_ms = now_ms;

        if (BSP_Oled_GetStatus(&oled) &&
            BSP_Key_GetStatus(&key))
        {
            (void)BSP_Debug_Printf(
                "OLED,ON=%u,ST=%s,ERR=%lu,"
                "DISC=%lu,RECON=%lu,RETRY=%lu\r\n",
                oled.online ? 1U : 0U,
                BSP_Oled_StateName(oled.state),
                (unsigned long)oled.error_count,
                (unsigned long)oled.disconnect_count,
                (unsigned long)oled.reconnect_count,
                (unsigned long)oled.retry_count);

            (void)BSP_Debug_Printf(
                "KEY,UP=%u,K0=%u,UP_CNT=%lu,K0_CNT=%lu\r\n",
                key.select_pressed ? 1U : 0U,
                key.confirm_pressed ? 1U : 0U,
                (unsigned long)key.select_press_count,
                (unsigned long)key.confirm_press_count);
        }
    }
}

void Test_OledKey_Stop(void)
{
    s_initialized = false;
    BSP_Oled_Clear();

    (void)BSP_Debug_Printf(
        "TEST,OLED_KEY,STOP\r\n");
}
