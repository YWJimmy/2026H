#include "app_task_port.h"

#include "ball_balance_control.h"
#include "bsp_servo.h"
#include "chassis.h"
#include "line_follow_control.h"
#include "line_sensor.h"
#include "task2_lap_stop.h"

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

    if (task != TASK_MENU_TASK_2_LAP_STOP)
    {
        s_fault_detail = 100U + (uint32_t)task;
        return false;
    }

    s_task = task;
    s_fault_detail = 0U;
    s_active = Task2LapStop_Start(start_timestamp_ms);
    if (!s_active)
    {
        s_fault_detail = Task2LapStop_GetFaultDetail();
    }
    return s_active;
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
    Task2LapStopResult_t result;

    if ((!s_initialized) || (task != s_task))
    {
        return APP_TASK_PORT_RESULT_FAULT;
    }

    if (!s_active)
    {
        return Task2LapStop_IsStopped() ?
            APP_TASK_PORT_RESULT_FINISHED :
            APP_TASK_PORT_RESULT_FAULT;
    }

    result = Task2LapStop_Update(now_ms);
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

bool AppTaskPort_RequestStop(
    TaskMenuTask_t task)
{
    if ((!s_initialized) ||
        (task != s_task))
    {
        return false;
    }

    if (!Task2LapStop_RequestStop())
    {
        s_fault_detail = Task2LapStop_GetFaultDetail();
        return false;
    }

    if (Task2LapStop_IsStopped())
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

    if (Task2LapStop_IsStopped())
    {
        s_active = false;
    }
    return !s_active;
}

void AppTaskPort_ForceSafeStop(void)
{
    s_active = false;
    Task2LapStop_ForceSafeStop();

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
    return "TASK UNSUPPORTED";
}

uint32_t AppTaskPort_GetFaultDetail(void)
{
    return s_fault_detail;
}
