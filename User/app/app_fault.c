#include "app_fault.h"

#include "stm32f4xx_hal.h"

#include <stddef.h>

static AppFaultStatus_t s_fault;

void AppFault_Reset(void)
{
    s_fault.active = false;
    s_fault.code = APP_FAULT_NONE;
    s_fault.detail = 0U;
    s_fault.timestamp_ms = 0U;
}

void AppFault_Raise(
    AppFaultCode_t code,
    uint32_t detail)
{
    if ((code == APP_FAULT_NONE) ||
        s_fault.active)
    {
        return;
    }

    s_fault.active = true;
    s_fault.code = code;
    s_fault.detail = detail;
    s_fault.timestamp_ms = HAL_GetTick();
}

void AppFault_Clear(void)
{
    AppFault_Reset();
}

bool AppFault_IsActive(void)
{
    return s_fault.active;
}

AppFaultCode_t AppFault_GetCode(void)
{
    return s_fault.code;
}

bool AppFault_GetStatus(AppFaultStatus_t *status)
{
    if (status == NULL)
    {
        return false;
    }

    *status = s_fault;
    return true;
}

const char *AppFault_Name(AppFaultCode_t code)
{
    switch (code)
    {
        case APP_FAULT_NONE:
            return "NONE";

        case APP_FAULT_DEBUG_UART_INIT:
            return "DBG INIT";

        case APP_FAULT_KEY_INIT:
            return "KEY INIT";

        case APP_FAULT_OLED_INIT:
            return "OLED INIT";

        case APP_FAULT_MENU_INIT:
            return "MENU INIT";

        case APP_FAULT_TASK_MANAGER_INIT:
            return "TASK INIT";

        case APP_FAULT_TASK_START:
            return "TASK START";

        case APP_FAULT_TASK_RUNTIME:
            return "TASK RUN";

        case APP_FAULT_TASK_STOP_TIMEOUT:
            return "STOP TIMEOUT";

        default:
            return "UNKNOWN";
    }
}
