#ifndef APP_FAULT_H
#define APP_FAULT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_FAULT_NONE = 0,
    APP_FAULT_DEBUG_UART_INIT,
    APP_FAULT_KEY_INIT,
    APP_FAULT_OLED_INIT,
    APP_FAULT_MENU_INIT,
    APP_FAULT_TASK_MANAGER_INIT,
    APP_FAULT_TASK_START,
    APP_FAULT_TASK_RUNTIME,
    APP_FAULT_TASK_STOP_TIMEOUT
} AppFaultCode_t;

typedef struct
{
    bool active;
    AppFaultCode_t code;
    uint32_t detail;
    uint32_t timestamp_ms;
} AppFaultStatus_t;

void AppFault_Reset(void);
void AppFault_Raise(
    AppFaultCode_t code,
    uint32_t detail);
void AppFault_Clear(void);

bool AppFault_IsActive(void);
AppFaultCode_t AppFault_GetCode(void);
bool AppFault_GetStatus(AppFaultStatus_t *status);

const char *AppFault_Name(AppFaultCode_t code);

#ifdef __cplusplus
}
#endif

#endif /* APP_FAULT_H */
