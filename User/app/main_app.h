#ifndef MAIN_APP_H
#define MAIN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "app_fault.h"
#include "task_menu_ui.h"

typedef enum
{
    APP_STATE_BOOT = 0,
    APP_STATE_SELF_CHECK,
    APP_STATE_MENU,
    APP_STATE_ARMED,
    APP_STATE_STARTING,
    APP_STATE_RUNNING,
    APP_STATE_STOPPING,
    APP_STATE_FINISHED,
    APP_STATE_FAULT
} AppState_t;

typedef struct
{
    bool initialized;

    AppState_t state;
    TaskMenuTask_t selected_task;

    uint32_t state_timestamp_ms;
    uint32_t run_start_timestamp_ms;
    uint32_t elapsed_ms;
    uint32_t warning_mask;

    AppFaultCode_t fault_code;
    uint32_t fault_detail;
} MainAppStatus_t;

bool MainApp_Init(void);
void MainApp_Update(void);
void MainApp_Shutdown(void);

bool MainApp_GetStatus(
    MainAppStatus_t *status);

const char *MainApp_StateName(
    AppState_t state);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_APP_H */
