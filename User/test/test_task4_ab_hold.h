#ifndef TEST_TASK4_AB_HOLD_H
#define TEST_TASK4_AB_HOLD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool Test_Task4ABHold_Init(void);
void Test_Task4ABHold_Update(void);
void Test_Task4ABHold_Stop(void);
bool Test_Task4ABHold_IsInitialized(void);
bool Test_Task4ABHold_IsFinished(void);
bool Test_Task4ABHold_IsPassed(void);
uint32_t Test_Task4ABHold_GetElapsedMs(void);
bool Test_Task4ABHold_IsTimerRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_TASK4_AB_HOLD_H */
