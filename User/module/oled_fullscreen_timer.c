#include "oled_fullscreen_timer.h"

#include "bsp_oled.h"

#include <stdbool.h>

#define TIMER_MAX_DECISECONDS       ((uint32_t)999U)

#define TIMER_DIGIT_Y               ((uint8_t)4U)
#define TIMER_DIGIT_WIDTH           ((uint8_t)26U)
#define TIMER_DIGIT_HEIGHT          ((uint8_t)56U)
#define TIMER_SEGMENT_THICKNESS     ((uint8_t)5U)
#define TIMER_VERTICAL_HEIGHT       ((uint8_t)20U)
#define TIMER_HORIZONTAL_WIDTH      ((uint8_t)16U)

#define TIMER_TENS_X                ((uint8_t)14U)
#define TIMER_ONES_X                ((uint8_t)44U)
#define TIMER_DOT_X                 ((uint8_t)75U)
#define TIMER_TENTHS_X              ((uint8_t)86U)
#define TIMER_DOT_Y                 ((uint8_t)52U)
#define TIMER_DOT_SIZE              ((uint8_t)7U)

#define TIMER_SEG_A                 ((uint8_t)(1U << 0U))
#define TIMER_SEG_B                 ((uint8_t)(1U << 1U))
#define TIMER_SEG_C                 ((uint8_t)(1U << 2U))
#define TIMER_SEG_D                 ((uint8_t)(1U << 3U))
#define TIMER_SEG_E                 ((uint8_t)(1U << 4U))
#define TIMER_SEG_F                 ((uint8_t)(1U << 5U))
#define TIMER_SEG_G                 ((uint8_t)(1U << 6U))

static bool s_force_repaint = true;
static uint32_t s_last_deciseconds = 0U;

static uint8_t OledTimer_GetSegments(uint8_t digit)
{
    static const uint8_t segment_map[10] =
    {
        0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
        0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
    };

    return segment_map[digit % 10U];
}

static void OledTimer_DrawDigit(uint8_t x, uint8_t digit)
{
    uint8_t segments = OledTimer_GetSegments(digit);
    uint8_t right_x = (uint8_t)(
        x + TIMER_DIGIT_WIDTH -
        TIMER_SEGMENT_THICKNESS);
    uint8_t middle_y = (uint8_t)(
        TIMER_DIGIT_Y +
        (TIMER_DIGIT_HEIGHT / 2U) -
        (TIMER_SEGMENT_THICKNESS / 2U));
    uint8_t lower_y = (uint8_t)(
        middle_y + TIMER_SEGMENT_THICKNESS);

    if ((segments & TIMER_SEG_A) != 0U)
    {
        BSP_Oled_FillRect(
            (uint8_t)(x + TIMER_SEGMENT_THICKNESS),
            TIMER_DIGIT_Y,
            TIMER_HORIZONTAL_WIDTH,
            TIMER_SEGMENT_THICKNESS,
            true);
    }
    if ((segments & TIMER_SEG_B) != 0U)
    {
        BSP_Oled_FillRect(
            right_x,
            (uint8_t)(TIMER_DIGIT_Y +
                TIMER_SEGMENT_THICKNESS),
            TIMER_SEGMENT_THICKNESS,
            TIMER_VERTICAL_HEIGHT,
            true);
    }
    if ((segments & TIMER_SEG_C) != 0U)
    {
        BSP_Oled_FillRect(
            right_x,
            lower_y,
            TIMER_SEGMENT_THICKNESS,
            TIMER_VERTICAL_HEIGHT,
            true);
    }
    if ((segments & TIMER_SEG_D) != 0U)
    {
        BSP_Oled_FillRect(
            (uint8_t)(x + TIMER_SEGMENT_THICKNESS),
            (uint8_t)(TIMER_DIGIT_Y +
                TIMER_DIGIT_HEIGHT -
                TIMER_SEGMENT_THICKNESS),
            TIMER_HORIZONTAL_WIDTH,
            TIMER_SEGMENT_THICKNESS,
            true);
    }
    if ((segments & TIMER_SEG_E) != 0U)
    {
        BSP_Oled_FillRect(
            x,
            lower_y,
            TIMER_SEGMENT_THICKNESS,
            TIMER_VERTICAL_HEIGHT,
            true);
    }
    if ((segments & TIMER_SEG_F) != 0U)
    {
        BSP_Oled_FillRect(
            x,
            (uint8_t)(TIMER_DIGIT_Y +
                TIMER_SEGMENT_THICKNESS),
            TIMER_SEGMENT_THICKNESS,
            TIMER_VERTICAL_HEIGHT,
            true);
    }
    if ((segments & TIMER_SEG_G) != 0U)
    {
        BSP_Oled_FillRect(
            (uint8_t)(x + TIMER_SEGMENT_THICKNESS),
            middle_y,
            TIMER_HORIZONTAL_WIDTH,
            TIMER_SEGMENT_THICKNESS,
            true);
    }
}

void OledFullscreenTimer_Reset(void)
{
    s_force_repaint = true;
}

void OledFullscreenTimer_ShowElapsedMs(uint32_t elapsed_ms)
{
    uint32_t deciseconds = elapsed_ms / 100U;
    uint8_t whole_seconds;

    if (deciseconds > TIMER_MAX_DECISECONDS)
    {
        deciseconds = TIMER_MAX_DECISECONDS;
    }
    if (!s_force_repaint &&
        (deciseconds == s_last_deciseconds))
    {
        return;
    }

    s_force_repaint = false;
    s_last_deciseconds = deciseconds;
    whole_seconds = (uint8_t)(deciseconds / 10U);

    BSP_Oled_Clear();
    OledTimer_DrawDigit(
        TIMER_TENS_X,
        (uint8_t)(whole_seconds / 10U));
    OledTimer_DrawDigit(
        TIMER_ONES_X,
        (uint8_t)(whole_seconds % 10U));
    BSP_Oled_FillRect(
        TIMER_DOT_X,
        TIMER_DOT_Y,
        TIMER_DOT_SIZE,
        TIMER_DOT_SIZE,
        true);
    OledTimer_DrawDigit(
        TIMER_TENTHS_X,
        (uint8_t)(deciseconds % 10U));
}
