#ifndef BSP_OLED_H
#define BSP_OLED_H
#include <stdbool.h>
#include <stdint.h>
#define BSP_OLED_WIDTH 128U
#define BSP_OLED_HEIGHT 64U
typedef int BspOledState_t;
typedef struct { bool online; BspOledState_t state; uint8_t dirty_mask; uint32_t error_count,disconnect_count,reconnect_count,retry_count,transfer_count,last_error; } BspOledStatus_t;
bool BSP_Oled_Init(void); void BSP_Oled_Process(void); void BSP_Oled_Clear(void); void BSP_Oled_DrawString(uint8_t x,uint8_t page,const char *s); void BSP_Oled_DrawU32(uint8_t x,uint8_t page,uint32_t value); void BSP_Oled_SetPixel(uint8_t x,uint8_t y,bool on); void BSP_Oled_FillRect(uint8_t x,uint8_t y,uint8_t w,uint8_t h,bool on); bool BSP_Oled_IsOnline(void); bool BSP_Oled_GetStatus(BspOledStatus_t *status); const char *BSP_Oled_StateName(BspOledState_t state);
#endif
