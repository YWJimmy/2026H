#include "test_oled_key.h"

#include "bsp_debug_uart.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "task_menu_ui.h"
#include "test_ball_balance.h"
#include "test_task4_ab_hold.h"

#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_OLED_KEY_REPORT_MS        1000U

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;
static bool s_task_active = false;
static bool s_task_finish_reported = false;
static TaskMenuTask_t s_active_task = TASK_MENU_TASK_2_LAP_STOP;

static TaskMenuTask_t s_last_task =
    TASK_MENU_TASK_2_LAP_STOP;
static TaskMenuState_t s_last_state =
    TASK_MENU_STATE_SELECT;
static bool s_last_oled_online = false;


static bool Test_OledKey_StartTask(TaskMenuTask_t task)
{
    if (s_task_active)
    {
        return false;
    }

    if (task == TASK_MENU_TASK_3_BALL_SEQUENCE)
    {
        if (!Test_BallBalance_Init())
        {
            (void)BSP_Debug_Printf(
                "ERR,TASK3_BALL_INIT\r\n");
            return false;
        }

        s_active_task = task;
        s_task_active = true;
        s_task_finish_reported = false;
        (void)BSP_Debug_Printf(
            "UI,EVENT=TASK_STARTED,TASK=3,CTRL=ONE_SHOT_PREDICT_CAPTURE\r\n");
        return true;
    }

    if (task == TASK_MENU_TASK_4_AB_HOLD)
    {
        if (!Test_Task4ABHold_Init())
        {
            (void)BSP_Debug_Printf(
                "ERR,TASK4_INIT\r\n");
            return false;
        }

        s_active_task = task;
        s_task_active = true;
        s_task_finish_reported = false;
        (void)BSP_Debug_Printf(
            "UI,EVENT=TASK_STARTED,TASK=4\r\n");
        return true;
    }

    (void)BSP_Debug_Printf(
        "UI,EVENT=TASK_NOT_IMPLEMENTED,TASK=%u\r\n",
        (unsigned int)task);
    return false;
}

static void Test_OledKey_StopActiveTask(void)
{
    if (!s_task_active)
    {
        return;
    }

    if (s_active_task == TASK_MENU_TASK_3_BALL_SEQUENCE)
    {
        Test_BallBalance_Stop();
    }
    else if (s_active_task == TASK_MENU_TASK_4_AB_HOLD)
    {
        Test_Task4ABHold_Stop();
    }

    (void)BSP_Debug_Printf(
        "UI,EVENT=TASK_STOPPED,TASK=%u\r\n",
        (unsigned int)s_active_task);

    s_task_active = false;
    s_task_finish_reported = false;
    s_active_task = TASK_MENU_TASK_2_LAP_STOP;
}

static void Test_OledKey_UpdateActiveTask(void)
{
    if (!s_task_active)
    {
        return;
    }

    if (s_active_task == TASK_MENU_TASK_3_BALL_SEQUENCE)
    {
        Test_BallBalance_Update();
        TaskMenuUi_SetRunningElapsedMs(
            Test_BallBalance_GetElapsedMs(),
            Test_BallBalance_IsTimerRunning());

        if (Test_BallBalance_IsFinished() &&
            (!s_task_finish_reported))
        {
            s_task_finish_reported = true;
            TaskMenuUi_SetFinishedResult(
                Test_BallBalance_Passed(),
                Test_BallBalance_GetElapsedMs());
            (void)BSP_Debug_Printf(
                "UI,EVENT=TASK_FINISHED,TASK=3,RESULT=%s,"
                "HOLDING_NEG5_UNTIL_K0\r\n",
                Test_BallBalance_Passed() ? "PASS" : "FAIL");
        }
    }
    else if (s_active_task == TASK_MENU_TASK_4_AB_HOLD)
    {
        Test_Task4ABHold_Update();
        TaskMenuUi_SetRunningElapsedMs(
            Test_Task4ABHold_GetElapsedMs(),
            Test_Task4ABHold_IsTimerRunning());

        if (Test_Task4ABHold_IsFinished() &&
            (!s_task_finish_reported))
        {
            s_task_finish_reported = true;
            TaskMenuUi_SetFinishedResult(
                Test_Task4ABHold_IsPassed(),
                Test_Task4ABHold_GetElapsedMs());
            (void)BSP_Debug_Printf(
                "UI,EVENT=TASK_FINISHED,TASK=4,RESULT=%s\r\n",
                Test_Task4ABHold_IsPassed() ? "PASS" : "FAIL");
        }
    }
}

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
    s_task_active = false;
    s_task_finish_reported = false;
    s_active_task = TASK_MENU_TASK_2_LAP_STOP;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_Key_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,KEY_INIT\r\n");
        return false;
    }

    if (!BSP_Oled_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,OLED_INIT,CHECK=I2C1_400KHZ_IRQ\r\n");
        return false;
    }

    if (!TaskMenuUi_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,TASK_MENU_INIT\r\n");
        return false;
    }

    s_last_task =
        TaskMenuUi_GetSelectedTask();
    s_last_state =
        TaskMenuUi_GetState();
    s_last_oled_online = false;
    s_last_report_ms = HAL_GetTick();

    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,OLED_KEY,START,"
        "OLED=SSD1306_128X64_ADDR_0X3C,"
        "I2C1=400KHZ_IT\r\n");

    (void)BSP_Debug_Printf(
        "KEY,SELECT=KEY_UP_PA0_HIGH,"
        "CONFIRM=KEY0_PE4_HIGH,"
        "LONG_PRESS=DISABLED\r\n");

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

    BSP_Key_Process();
    TaskMenuUi_Process();

    if (TaskMenuUi_TakeStartRequest(
            &start_task))
    {
        (void)BSP_Debug_Printf(
            "UI,EVENT=START_REQUEST,TASK=%u,NAME=%s\r\n",
            (unsigned int)start_task,
            TaskMenuUi_TaskName(start_task));

        if (!Test_OledKey_StartTask(start_task))
        {
            TaskMenuUi_SetFinished();
        }
    }

    if (TaskMenuUi_TakeStopRequest())
    {
        (void)BSP_Debug_Printf(
            "UI,EVENT=STOP_REQUEST\r\n");
        Test_OledKey_StopActiveTask();
    }

    Test_OledKey_UpdateActiveTask();
    BSP_Oled_Process();
    Test_OledKey_ReportChange();

    now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - s_last_report_ms) >=
        TEST_OLED_KEY_REPORT_MS)
    {
        s_last_report_ms = now_ms;

        if (BSP_Oled_GetStatus(&oled) &&
            BSP_Key_GetStatus(&key))
        {
            (void)BSP_Debug_Printf(
                "OLED,ON=%u,ST=%s,DIRTY=0x%02X,"
                "ERR=%lu,DISC=%lu,RECON=%lu,"
                "RETRY=%lu,TX=%lu,LAST=0x%08lX\r\n",
                oled.online ? 1U : 0U,
                BSP_Oled_StateName(oled.state),
                (unsigned int)oled.dirty_mask,
                (unsigned long)oled.error_count,
                (unsigned long)oled.disconnect_count,
                (unsigned long)oled.reconnect_count,
                (unsigned long)oled.retry_count,
                (unsigned long)oled.transfer_count,
                (unsigned long)oled.last_error);

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
    Test_OledKey_StopActiveTask();
    s_initialized = false;
    BSP_Oled_Clear();

    (void)BSP_Debug_Printf(
        "TEST,OLED_KEY,STOP\r\n");
}
