#ifndef TEST_CHASSIS_RAMP_H
#define TEST_CHASSIS_RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_ChassisRamp_Init(void);
void Test_ChassisRamp_Update(void);
void Test_ChassisRamp_Stop(void);
bool Test_ChassisRamp_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_CHASSIS_RAMP_H */
