#ifndef OLED_FULLSCREEN_TIMER_H
#define OLED_FULLSCREEN_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Force the next Show call to repaint the complete 128x64 timer page. */
void OledFullscreenTimer_Reset(void);

/*
 * Display elapsed time as XX.X with no unit. Values above 99.9 s saturate.
 * The framebuffer is repainted only when the displayed decisecond changes.
 */
void OledFullscreenTimer_ShowElapsedMs(uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* OLED_FULLSCREEN_TIMER_H */
