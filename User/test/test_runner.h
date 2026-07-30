#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool TestRunner_Init(void);
void TestRunner_Update(void);
void TestRunner_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_RUNNER_H */
