#ifndef BSP_DEBUG_UART_H
#define BSP_DEBUG_UART_H
#include <stdbool.h>
bool BSP_DebugUart_Init(void);
void BSP_DebugUart_Process(void);
int BSP_Debug_Printf(const char *fmt, ...);
#endif
