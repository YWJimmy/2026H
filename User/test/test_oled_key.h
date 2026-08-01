#ifndef TEST_OLED_KEY_H
#define TEST_OLED_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_OledKey_Init(void);
void Test_OledKey_Update(void);
void Test_OledKey_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_OLED_KEY_H */
