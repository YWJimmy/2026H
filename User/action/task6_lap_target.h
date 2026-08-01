#ifndef TASK6_LAP_TARGET_H
#define TASK6_LAP_TARGET_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TASK6_LAP_TARGET_RESULT_RUNNING = 0,
    TASK6_LAP_TARGET_RESULT_FINISHED,
    TASK6_LAP_TARGET_RESULT_FAULT
} Task6LapTargetResult_t;

bool Task6LapTarget_Init(void);
bool Task6LapTarget_Start(uint32_t start_timestamp_ms);
Task6LapTargetResult_t Task6LapTarget_Update(uint32_t now_ms);
bool Task6LapTarget_RequestStop(void);
bool Task6LapTarget_IsStopped(void);
void Task6LapTarget_ForceSafeStop(void);
bool Task6LapTarget_GetElapsedMs(uint32_t now_ms, uint32_t *elapsed_ms);
const char *Task6LapTarget_GetPhaseText(void);
uint32_t Task6LapTarget_GetFaultDetail(void);

#ifdef __cplusplus
}
#endif
#endif /* TASK6_LAP_TARGET_H */
