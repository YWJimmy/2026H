#include "bsp_key.h"

#include "main.h"
#include "stm32f4xx_hal.h"

#include <stddef.h>

#define BSP_KEY_DEBOUNCE_MS          30U
#define BSP_KEY_EVENT_MAX            255U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;

    bool raw_pressed;
    bool stable_pressed;

    uint8_t pending_press_count;

    uint32_t raw_change_ms;
    uint32_t total_press_count;
} KeyChannel_t;

static bool s_initialized = false;

static KeyChannel_t s_keys[BSP_KEY_COUNT];

static bool Key_ReadRawPressed(
    const KeyChannel_t *key)
{
    return (HAL_GPIO_ReadPin(
                key->port,
                key->pin) == GPIO_PIN_SET);
}

static void Key_InitChannel(
    KeyChannel_t *key,
    GPIO_TypeDef *port,
    uint16_t pin,
    uint32_t now_ms)
{
    key->port = port;
    key->pin = pin;

    key->raw_pressed =
        (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET);
    key->stable_pressed = key->raw_pressed;

    key->pending_press_count = 0U;
    key->raw_change_ms = now_ms;
    key->total_press_count = 0U;
}

bool BSP_Key_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    Key_InitChannel(
        &s_keys[BSP_KEY_SELECT],
        KEY_UP_GPIO_Port,
        KEY_UP_Pin,
        now_ms);

    Key_InitChannel(
        &s_keys[BSP_KEY_CONFIRM],
        KEY0_GPIO_Port,
        KEY0_Pin,
        now_ms);

    s_initialized = true;
    return true;
}

void BSP_Key_Process(void)
{
    uint32_t now_ms;
    uint8_t index;

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    for (index = 0U;
         index < (uint8_t)BSP_KEY_COUNT;
         index++)
    {
        KeyChannel_t *key = &s_keys[index];
        bool raw_pressed =
            Key_ReadRawPressed(key);

        if (raw_pressed != key->raw_pressed)
        {
            key->raw_pressed = raw_pressed;
            key->raw_change_ms = now_ms;
            continue;
        }

        if ((raw_pressed != key->stable_pressed) &&
            ((uint32_t)(now_ms - key->raw_change_ms) >=
             BSP_KEY_DEBOUNCE_MS))
        {
            key->stable_pressed = raw_pressed;

            if (raw_pressed)
            {
                if (key->pending_press_count <
                    BSP_KEY_EVENT_MAX)
                {
                    key->pending_press_count++;
                }

                key->total_press_count++;
            }
        }
    }
}

bool BSP_Key_TakePress(BspKeyId_t key)
{
    if ((!s_initialized) ||
        (key >= BSP_KEY_COUNT))
    {
        return false;
    }

    if (s_keys[key].pending_press_count == 0U)
    {
        return false;
    }

    s_keys[key].pending_press_count--;
    return true;
}

bool BSP_Key_IsPressed(BspKeyId_t key)
{
    if ((!s_initialized) ||
        (key >= BSP_KEY_COUNT))
    {
        return false;
    }

    return s_keys[key].stable_pressed;
}

bool BSP_Key_IsInitialized(void)
{
    return s_initialized;
}

bool BSP_Key_GetStatus(BspKeyStatus_t *status)
{
    if ((!s_initialized) || (status == NULL))
    {
        return false;
    }

    status->initialized = s_initialized;

    status->select_pressed =
        s_keys[BSP_KEY_SELECT].stable_pressed;
    status->confirm_pressed =
        s_keys[BSP_KEY_CONFIRM].stable_pressed;

    status->select_press_count =
        s_keys[BSP_KEY_SELECT].total_press_count;
    status->confirm_press_count =
        s_keys[BSP_KEY_CONFIRM].total_press_count;

    return true;
}
