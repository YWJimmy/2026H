#ifndef TASK2_LAP_STOP_H
#define TASK2_LAP_STOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TASK2_LAP_STOP_RESULT_RUNNING = 0,
    TASK2_LAP_STOP_RESULT_FINISHED,
    TASK2_LAP_STOP_RESULT_FAULT
} Task2LapStopResult_t;

bool Task2LapStop_Init(void);
bool Task2LapStop_Start(uint32_t start_timestamp_ms);
Task2LapStopResult_t Task2LapStop_Update(uint32_t now_ms);
bool Task2LapStop_RequestStop(void);
bool Task2LapStop_IsStopped(void);
void Task2LapStop_ForceSafeStop(void);

const char *Task2LapStop_GetPhaseText(void);
uint32_t Task2LapStop_GetFaultDetail(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK2_LAP_STOP_H */
