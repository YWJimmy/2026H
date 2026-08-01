#ifndef TEST_BALL_BALANCE_H
#define TEST_BALL_BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool Test_BallBalance_Init(void);
void Test_BallBalance_Update(void);
void Test_BallBalance_Stop(void);
bool Test_BallBalance_IsInitialized(void);
bool Test_BallBalance_IsFinished(void);
bool Test_BallBalance_Passed(void);
bool Test_BallBalance_IsTimerRunning(void);
uint32_t Test_BallBalance_GetElapsedMs(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BALL_BALANCE_H */
