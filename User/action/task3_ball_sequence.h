#ifndef TASK3_BALL_SEQUENCE_H
#define TASK3_BALL_SEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TASK3_BALL_SEQUENCE_RESULT_RUNNING = 0,
    TASK3_BALL_SEQUENCE_RESULT_FINISHED,
    TASK3_BALL_SEQUENCE_RESULT_FAULT
} Task3BallSequenceResult_t;

bool Task3BallSequence_Init(void);
bool Task3BallSequence_Start(uint32_t start_timestamp_ms);
Task3BallSequenceResult_t Task3BallSequence_Update(uint32_t now_ms);
bool Task3BallSequence_Maintain(uint32_t now_ms);
bool Task3BallSequence_RequestStop(void);
bool Task3BallSequence_IsStopped(void);
void Task3BallSequence_ForceSafeStop(void);
bool Task3BallSequence_GetElapsedMs(
    uint32_t now_ms,
    uint32_t *elapsed_ms);
const char *Task3BallSequence_GetPhaseText(void);
uint32_t Task3BallSequence_GetFaultDetail(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK3_BALL_SEQUENCE_H */
