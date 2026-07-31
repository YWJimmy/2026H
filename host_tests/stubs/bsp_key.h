#ifndef BSP_KEY_H
#define BSP_KEY_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { BSP_KEY_SELECT=0, BSP_KEY_CONFIRM=1 } BspKeyId_t;
typedef struct { bool select_pressed,confirm_pressed; uint32_t select_press_count,confirm_press_count; } BspKeyStatus_t;
bool BSP_Key_Init(void); void BSP_Key_Process(void); bool BSP_Key_TakePress(BspKeyId_t key); bool BSP_Key_GetStatus(BspKeyStatus_t *status);
#endif
