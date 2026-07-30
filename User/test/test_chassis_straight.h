#ifndef TEST_CHASSIS_STRAIGHT_H
#define TEST_CHASSIS_STRAIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_ChassisStraight_Init(void);
void Test_ChassisStraight_Update(void);
void Test_ChassisStraight_Stop(void);
bool Test_ChassisStraight_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_CHASSIS_STRAIGHT_H */
