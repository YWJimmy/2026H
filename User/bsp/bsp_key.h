#ifndef BSP_KEY_H
#define BSP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_KEY_SELECT = 0,   /* KEY_UP / PA0 / 高电平按下 */
    BSP_KEY_CONFIRM,      /* KEY0   / PE4 / 高电平按下 */
    BSP_KEY_COUNT
} BspKeyId_t;

typedef struct
{
    bool initialized;

    bool select_pressed;
    bool confirm_pressed;

    uint32_t select_press_count;
    uint32_t confirm_press_count;
} BspKeyStatus_t;

/**
 * @brief 初始化两个板载按键。
 *
 * 当前CubeMX配置：
 * - KEY_UP：PA0，GPIO_INPUT，PULLDOWN，高电平按下
 * - KEY0：PE4，GPIO_INPUT，PULLDOWN，高电平按下
 */
bool BSP_Key_Init(void);

/**
 * @brief 非阻塞按键采样与30 ms消抖。
 *
 * 主循环持续调用。不提供长按功能。
 */
void BSP_Key_Process(void);

/**
 * @brief 取走一次按下事件。
 */
bool BSP_Key_TakePress(BspKeyId_t key);

/**
 * @brief 查询当前稳定按下状态。
 */
bool BSP_Key_IsPressed(BspKeyId_t key);

bool BSP_Key_IsInitialized(void);
bool BSP_Key_GetStatus(BspKeyStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_H */
