#ifndef TEST_MOTOR_OPEN_LOOP_H
#define TEST_MOTOR_OPEN_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_MotorOpenLoop_Init(void);
void Test_MotorOpenLoop_Update(void);
void Test_MotorOpenLoop_Stop(void);
bool Test_MotorOpenLoop_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_MOTOR_OPEN_LOOP_H */
