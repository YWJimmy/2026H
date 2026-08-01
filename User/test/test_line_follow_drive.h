#ifndef TEST_LINE_FOLLOW_DRIVE_H
#define TEST_LINE_FOLLOW_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_LineFollowDrive_Init(void);
void Test_LineFollowDrive_Update(void);
void Test_LineFollowDrive_Stop(void);
bool Test_LineFollowDrive_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_LINE_FOLLOW_DRIVE_H */
