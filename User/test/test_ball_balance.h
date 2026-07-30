#ifndef TEST_BALL_BALANCE_H
#define TEST_BALL_BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_BallBalance_Init(void);
void Test_BallBalance_Update(void);
void Test_BallBalance_Stop(void);
bool Test_BallBalance_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BALL_BALANCE_H */
