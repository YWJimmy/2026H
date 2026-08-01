#ifndef TEST_WHEEL_SPEED_H
#define TEST_WHEEL_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_WheelSpeed_Init(void);
void Test_WheelSpeed_Update(void);
void Test_WheelSpeed_Stop(void);
bool Test_WheelSpeed_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_WHEEL_SPEED_H */
