#ifndef TEST_VISION_UART_H
#define TEST_VISION_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Test_VisionUart_Init(void);
void Test_VisionUart_Update(void);
void Test_VisionUart_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_VISION_UART_H */
