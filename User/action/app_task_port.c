#include "app_task_port.h"

#include "ball_balance_control.h"
#include "bsp_servo.h"
#include "chassis.h"
#include "line_follow_control.h"
#include "line_sensor.h"
#include "task2_lap_stop.h"
#include "task4_ab_hold.h"
#include "task5_lap_hold.h"
#include "task6_lap_target.h"

static bool s_initialized = false;
static bool s_active = false;
static TaskMenuTask_t s_task =
    TASK_MENU_TASK_2_LAP_STOP;
static uint32_t s_fault_detail = 0U;

bool AppTaskPort_Init(void)
{
    s_initialized = false;
    s_active = false;
    s_task = TASK_MENU_TASK_2_LAP_STOP;
    s_fault_detail = 0U;

    if (!Task2LapStop_Init())
    {
        s_fault_detail = Task2LapStop_GetFaultDetail();
        return false;
    }
    if (!Task4AbHold_Init())
    {
        s_fault_detail = Task4AbHold_GetFaultDetail();
        return false;
    }
    if (!Task5LapHold_Init())
    {
        s_fault_detail = Task5LapHold_GetFaultDetail();
        return false;
    }
    if (!Task6LapTarget_Init())
    {
        s_fault_detail = Task6LapTarget_GetFaultDetail();
        return false;
    }

    s_initialized = true;
    return true;
}

bool AppTaskPort_Start(
    TaskMenuTask_t task,
    uint32_t start_timestamp_ms)
{
    if (!s_initialized)
    {
        return false;
    }

    s_task = task;
    s_fault_detail = 0U;

    if (task == TASK_MENU_TASK_2_LAP_STOP)
    {
        s_active = Task2LapStop_Start(start_timestamp_ms);
        if (!s_active)
        {
            s_fault_detail = Task2LapStop_GetFaultDetail();
        }
        return s_active;
    }

    if (task == TASK_MENU_TASK_4_AB_HOLD)
    {
        s_active = Task4AbHold_Start(start_timestamp_ms);
        if (!s_active)
        {
            s_fault_detail = Task4AbHold_GetFaultDetail();
        }
        return s_active;
    }

    if (task == TASK_MENU_TASK_5_LAP_HOLD)
    {
        s_active = Task5LapHold_Start(start_timestamp_ms);
        if (!s_active)
        {
            s_fault_detail = Task5LapHold_GetFaultDetail();
        }
        return s_active;
    }

    if (task == TASK_MENU_TASK_6_LAP_TARGET)
    {
        s_active = Task6LapTarget_Start(start_timestamp_ms);
        if (!s_active)
        {
            s_fault_detail = Task6LapTarget_GetFaultDetail();
        }
        return s_active;
    }

    {
        s_fault_detail = 100U + (uint32_t)task;
        return false;
    }
}

void AppTaskPort_ProcessInputs(void)
{
    if (s_initialized && LineSensor_IsRunning())
    {
        (void)LineSensor_Update();
    }
}

AppTaskPortResult_t AppTaskPort_Update(
    TaskMenuTask_t task,
    uint32_t now_ms)
{
    if ((!s_initialized) || (task != s_task))
    {
        return APP_TASK_PORT_RESULT_FAULT;
    }

    if (!s_active)
    {
        if ((task == TASK_MENU_TASK_2_LAP_STOP) &&
            Task2LapStop_IsStopped())
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        if ((task == TASK_MENU_TASK_4_AB_HOLD) &&
            Task4AbHold_IsStopped())
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        if ((task == TASK_MENU_TASK_5_LAP_HOLD) &&
            Task5LapHold_IsStopped())
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        if ((task == TASK_MENU_TASK_6_LAP_TARGET) &&
            Task6LapTarget_IsStopped())
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        return APP_TASK_PORT_RESULT_FAULT;
    }

    if (task == TASK_MENU_TASK_2_LAP_STOP)
    {
        Task2LapStopResult_t result =
            Task2LapStop_Update(now_ms);

        if (result == TASK2_LAP_STOP_RESULT_FINISHED)
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        if (result == TASK2_LAP_STOP_RESULT_FAULT)
        {
            s_fault_detail = Task2LapStop_GetFaultDetail();
            return APP_TASK_PORT_RESULT_FAULT;
        }
        return APP_TASK_PORT_RESULT_RUNNING;
    }

    if (task == TASK_MENU_TASK_4_AB_HOLD)
    {
        Task4AbHoldResult_t result =
            Task4AbHold_Update(now_ms);

        if (result == TASK4_AB_HOLD_RESULT_FINISHED)
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        if (result == TASK4_AB_HOLD_RESULT_FAULT)
        {
            s_fault_detail = Task4AbHold_GetFaultDetail();
            return APP_TASK_PORT_RESULT_FAULT;
        }
        return APP_TASK_PORT_RESULT_RUNNING;
    }

    if (task == TASK_MENU_TASK_5_LAP_HOLD)
    {
        Task5LapHoldResult_t result =
            Task5LapHold_Update(now_ms);

        if (result == TASK5_LAP_HOLD_RESULT_FINISHED)
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        if (result == TASK5_LAP_HOLD_RESULT_FAULT)
        {
            s_fault_detail = Task5LapHold_GetFaultDetail();
            return APP_TASK_PORT_RESULT_FAULT;
        }
        return APP_TASK_PORT_RESULT_RUNNING;
    }

    if (task == TASK_MENU_TASK_6_LAP_TARGET)
    {
        Task6LapTargetResult_t result =
            Task6LapTarget_Update(now_ms);

        if (result == TASK6_LAP_TARGET_RESULT_FINISHED)
        {
            return APP_TASK_PORT_RESULT_FINISHED;
        }
        if (result == TASK6_LAP_TARGET_RESULT_FAULT)
        {
            s_fault_detail = Task6LapTarget_GetFaultDetail();
            return APP_TASK_PORT_RESULT_FAULT;
        }
        return APP_TASK_PORT_RESULT_RUNNING;
    }

    s_fault_detail = 100U + (uint32_t)task;
    return APP_TASK_PORT_RESULT_FAULT;
}

bool AppTaskPort_RequestStop(
    TaskMenuTask_t task)
{
    if ((!s_initialized) ||
        (task != s_task))
    {
        return false;
    }

    if ((task == TASK_MENU_TASK_2_LAP_STOP) &&
        !Task2LapStop_RequestStop())
    {
        s_fault_detail = Task2LapStop_GetFaultDetail();
        return false;
    }
    if ((task == TASK_MENU_TASK_4_AB_HOLD) &&
        !Task4AbHold_RequestStop())
    {
        s_fault_detail = Task4AbHold_GetFaultDetail();
        return false;
    }
    if ((task == TASK_MENU_TASK_5_LAP_HOLD) &&
        !Task5LapHold_RequestStop())
    {
        s_fault_detail = Task5LapHold_GetFaultDetail();
        return false;
    }
    if ((task == TASK_MENU_TASK_6_LAP_TARGET) &&
        !Task6LapTarget_RequestStop())
    {
        s_fault_detail = Task6LapTarget_GetFaultDetail();
        return false;
    }
    if ((task != TASK_MENU_TASK_2_LAP_STOP) &&
        (task != TASK_MENU_TASK_4_AB_HOLD) &&
        (task != TASK_MENU_TASK_5_LAP_HOLD) &&
        (task != TASK_MENU_TASK_6_LAP_TARGET))
    {
        s_fault_detail = 100U + (uint32_t)task;
        return false;
    }

    if (((task == TASK_MENU_TASK_2_LAP_STOP) &&
         Task2LapStop_IsStopped()) ||
        ((task == TASK_MENU_TASK_4_AB_HOLD) &&
         Task4AbHold_IsStopped()) ||
        ((task == TASK_MENU_TASK_5_LAP_HOLD) &&
         Task5LapHold_IsStopped()) ||
        ((task == TASK_MENU_TASK_6_LAP_TARGET) &&
         Task6LapTarget_IsStopped()))
    {
        s_active = false;
    }
    return true;
}

bool AppTaskPort_IsStopped(
    TaskMenuTask_t task)
{
    if ((!s_initialized) ||
        (task != s_task))
    {
        return false;
    }

    if (((task == TASK_MENU_TASK_2_LAP_STOP) &&
         Task2LapStop_IsStopped()) ||
        ((task == TASK_MENU_TASK_4_AB_HOLD) &&
         Task4AbHold_IsStopped()) ||
        ((task == TASK_MENU_TASK_5_LAP_HOLD) &&
         Task5LapHold_IsStopped()) ||
        ((task == TASK_MENU_TASK_6_LAP_TARGET) &&
         Task6LapTarget_IsStopped()))
    {
        s_active = false;
    }
    return !s_active;
}

void AppTaskPort_ForceSafeStop(void)
{
    s_active = false;
    Task2LapStop_ForceSafeStop();
    Task4AbHold_ForceSafeStop();
    Task5LapHold_ForceSafeStop();
    Task6LapTarget_ForceSafeStop();

    if (LineFollowControl_IsInitialized())
    {
        LineFollowControl_Shutdown();
    }

    if (BallBalanceControl_IsInitialized())
    {
        BallBalanceControl_Stop();
    }

    if (Chassis_IsInitialized())
    {
        Chassis_Stop();
        (void)Chassis_Enable(false);
    }

    if (BSP_Servo_IsInitialized())
    {
        BSP_Servo_Disable();
    }
}

void AppTaskPort_Reset(void)
{
    AppTaskPort_ForceSafeStop();
    s_task = TASK_MENU_TASK_2_LAP_STOP;
    s_fault_detail = 0U;
}

const char *AppTaskPort_GetPhaseText(void)
{
    if (s_task == TASK_MENU_TASK_2_LAP_STOP)
    {
        return Task2LapStop_GetPhaseText();
    }
    if (s_task == TASK_MENU_TASK_4_AB_HOLD)
    {
        return Task4AbHold_GetPhaseText();
    }
    if (s_task == TASK_MENU_TASK_5_LAP_HOLD)
    {
        return Task5LapHold_GetPhaseText();
    }
    if (s_task == TASK_MENU_TASK_6_LAP_TARGET)
    {
        return Task6LapTarget_GetPhaseText();
    }
    return "TASK UNSUPPORTED";
}

bool AppTaskPort_GetElapsedMs(
    TaskMenuTask_t task,
    uint32_t now_ms,
    uint32_t *elapsed_ms)
{
    if ((!s_initialized) || (task != s_task))
    {
        return false;
    }
    if (task == TASK_MENU_TASK_4_AB_HOLD)
    {
        return Task4AbHold_GetElapsedMs(
            now_ms,
            elapsed_ms);
    }
    if (task == TASK_MENU_TASK_5_LAP_HOLD)
    {
        return Task5LapHold_GetElapsedMs(
            now_ms,
            elapsed_ms);
    }
    if (task == TASK_MENU_TASK_6_LAP_TARGET)
    {
        return Task6LapTarget_GetElapsedMs(
            now_ms,
            elapsed_ms);
    }
    return false;
}

uint32_t AppTaskPort_GetFaultDetail(void)
{
    return s_fault_detail;
}
