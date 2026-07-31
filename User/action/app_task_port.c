#include "app_task_port.h"

#include "app_config.h"

static bool s_initialized = false;
static bool s_active = false;
static TaskMenuTask_t s_task =
    TASK_MENU_TASK_2_LAP_STOP;

bool AppTaskPort_Init(void)
{
    s_initialized = true;
    s_active = false;
    s_task = TASK_MENU_TASK_2_LAP_STOP;
    return true;
}

bool AppTaskPort_Start(
    TaskMenuTask_t task,
    uint32_t start_timestamp_ms)
{
    (void)start_timestamp_ms;

    if (!s_initialized)
    {
        return false;
    }

    if ((task < TASK_MENU_TASK_2_LAP_STOP) ||
        (task > TASK_MENU_TASK_6_LAP_TARGET))
    {
        return false;
    }

    s_task = task;
    s_active = true;

    /*
     * V1安全占位：
     * 不初始化或启动任何运动模块。
     */
    return true;
}

AppTaskPortResult_t AppTaskPort_Update(
    TaskMenuTask_t task,
    uint32_t now_ms)
{
    (void)now_ms;

    if ((!s_initialized) ||
        (!s_active) ||
        (task != s_task))
    {
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

    /*
     * 后续真实实现必须在这里请求底盘柔和停车、
     * 舵机安全回中等动作。
     */
    s_active = false;
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

    return !s_active;
}

void AppTaskPort_Reset(void)
{
    s_active = false;
    s_task = TASK_MENU_TASK_2_LAP_STOP;
}

const char *AppTaskPort_GetPhaseText(void)
{
#if APP_TASK_PORT_PLACEHOLDER
    return "SAFE SKELETON";
#else
    return "TASK ACTIVE";
#endif
}

uint32_t AppTaskPort_GetFaultDetail(void)
{
    return 0U;
}
