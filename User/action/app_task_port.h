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

/* Level and enable the ball rail while the selected task is armed. */
bool AppTaskPort_Prepare(TaskMenuTask_t task);

bool AppTaskPort_Start(
    TaskMenuTask_t task,
    uint32_t start_timestamp_ms);

/* Keep input devices serviced in every task-manager state. */
void AppTaskPort_ProcessInputs(void);

AppTaskPortResult_t AppTaskPort_Update(
    TaskMenuTask_t task,
    uint32_t now_ms);

/* Continue post-result actions such as ball position holding. */
bool AppTaskPort_Maintain(uint32_t now_ms);

bool AppTaskPort_RequestStop(
    TaskMenuTask_t task);

bool AppTaskPort_IsStopped(
    TaskMenuTask_t task);

/*
 * 幂等安全停车：立即撤销任务活动状态，并将所有已初始化执行机构
 * 带入安全状态。故障路径和系统关机均可重复调用。
 */
void AppTaskPort_ForceSafeStop(void);

void AppTaskPort_Reset(void);

const char *AppTaskPort_GetPhaseText(void);
uint32_t AppTaskPort_GetFaultDetail(void);
bool AppTaskPort_GetElapsedMs(
    TaskMenuTask_t task,
    uint32_t now_ms,
    uint32_t *elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_PORT_H */
