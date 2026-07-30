#ifndef TEST_LINE_UART_H
#define TEST_LINE_UART_H

#include <stdbool.h>

bool Test_LineUart_Init(void);
void Test_LineUart_Update(void);
void Test_LineUart_Stop(void);
bool Test_LineUart_IsInitialized(void);

#endif /* TEST_LINE_UART_H */
