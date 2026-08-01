#ifndef TASK4_AB_HOLD_H
#define TASK4_AB_HOLD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TASK4_AB_HOLD_RESULT_RUNNING = 0,
    TASK4_AB_HOLD_RESULT_FINISHED,
    TASK4_AB_HOLD_RESULT_FAULT
} Task4AbHoldResult_t;

bool Task4AbHold_Init(void);
bool Task4AbHold_Start(uint32_t start_timestamp_ms);
Task4AbHoldResult_t Task4AbHold_Update(uint32_t now_ms);
bool Task4AbHold_RequestStop(void);
bool Task4AbHold_IsStopped(void);
void Task4AbHold_ForceSafeStop(void);
bool Task4AbHold_GetElapsedMs(uint32_t now_ms, uint32_t *elapsed_ms);
const char *Task4AbHold_GetPhaseText(void);
uint32_t Task4AbHold_GetFaultDetail(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK4_AB_HOLD_H */
