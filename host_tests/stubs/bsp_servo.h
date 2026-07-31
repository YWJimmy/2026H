#ifndef BSP_SERVO_H
#define BSP_SERVO_H
#include <stdbool.h>
#include <stdint.h>
bool BSP_Servo_Init(void);
bool BSP_Servo_Enable(void);
void BSP_Servo_Disable(void);
bool BSP_Servo_IsEnabled(void);
bool BSP_Servo_SetPulseUs(uint16_t pulse_us);
uint16_t BSP_Servo_GetPulseUs(void);
#endif
