#ifndef TEST_OLED_FULLSCREEN_TIMER_H
#define TEST_OLED_FULLSCREEN_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_OledFullscreenTimer_Init(void);
void Test_OledFullscreenTimer_Update(void);
void Test_OledFullscreenTimer_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_OLED_FULLSCREEN_TIMER_H */
