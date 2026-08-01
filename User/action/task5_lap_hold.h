#ifndef TASK5_LAP_HOLD_H
#define TASK5_LAP_HOLD_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TASK5_LAP_HOLD_RESULT_RUNNING = 0,
    TASK5_LAP_HOLD_RESULT_FINISHED,
    TASK5_LAP_HOLD_RESULT_FAULT
} Task5LapHoldResult_t;

bool Task5LapHold_Init(void);
bool Task5LapHold_Start(uint32_t start_timestamp_ms);
Task5LapHoldResult_t Task5LapHold_Update(uint32_t now_ms);
bool Task5LapHold_RequestStop(void);
bool Task5LapHold_IsStopped(void);
void Task5LapHold_ForceSafeStop(void);
bool Task5LapHold_GetElapsedMs(uint32_t now_ms, uint32_t *elapsed_ms);
const char *Task5LapHold_GetPhaseText(void);
uint32_t Task5LapHold_GetFaultDetail(void);

#ifdef __cplusplus
}
#endif
#endif /* TASK5_LAP_HOLD_H */
