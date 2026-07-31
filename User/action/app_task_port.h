#ifndef APP_TASK_PORT_H
#define APP_TASK_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "task_menu_ui.h"

typedef enum
{
    APP_TASK_PORT_RESULT_RUNNING = 0,
    APP_TASK_PORT_RESULT_FINISHED,
    APP_TASK_PORT_RESULT_FAULT
} AppTaskPortResult_t;

/**
 * @brief 初始化正式任务适配端口。
 *
 * 当前V1为安全占位实现，不启动底盘、舵机、巡线和视觉控制。
 * 后续任务开发人员只需替换本文件实现，不修改MainApp。
 */
bool AppTaskPort_Init(void);

bool AppTaskPort_Start(
    TaskMenuTask_t task,
    uint32_t start_timestamp_ms);

AppTaskPortResult_t AppTaskPort_Update(
    TaskMenuTask_t task,
    uint32_t now_ms);

bool AppTaskPort_RequestStop(
    TaskMenuTask_t task);

bool AppTaskPort_IsStopped(
    TaskMenuTask_t task);

void AppTaskPort_Reset(void);

const char *AppTaskPort_GetPhaseText(void);
uint32_t AppTaskPort_GetFaultDetail(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_PORT_H */
