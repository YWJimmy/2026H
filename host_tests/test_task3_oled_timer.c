#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_key.h"
#include "bsp_oled.h"
#include "task_menu_ui.h"

static bool g_select_press;
static bool g_confirm_press;
static uint32_t g_clear_count;
static uint32_t g_fill_count;
static char g_state_text[8];

bool BSP_Key_Init(void) { return true; }
void BSP_Key_Process(void) { }
bool BSP_Key_TakePress(BspKeyId_t key)
{
    bool value;
    if (key == BSP_KEY_SELECT)
    {
        value = g_select_press;
        g_select_press = false;
        return value;
    }
    value = g_confirm_press;
    g_confirm_press = false;
    return value;
}
bool BSP_Key_GetStatus(BspKeyStatus_t *status) { (void)status; return true; }

bool BSP_Oled_Init(void) { return true; }
void BSP_Oled_Process(void) { }
void BSP_Oled_Clear(void) { g_clear_count++; g_fill_count = 0U; g_state_text[0] = '\0'; }
void BSP_Oled_DrawString(uint8_t x, uint8_t page, const char *s)
{
    if ((x == 94U) && (page == 0U))
    {
        (void)snprintf(g_state_text, sizeof(g_state_text), "%s", s);
    }
}
void BSP_Oled_DrawU32(uint8_t x, uint8_t page, uint32_t value)
{ (void)x; (void)page; (void)value; }
void BSP_Oled_SetPixel(uint8_t x, uint8_t y, bool on)
{ (void)x; (void)y; (void)on; }
void BSP_Oled_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{ (void)x; (void)y; (void)w; (void)h; (void)on; g_fill_count++; }
bool BSP_Oled_IsOnline(void) { return true; }
bool BSP_Oled_GetStatus(BspOledStatus_t *status) { (void)status; return true; }
const char *BSP_Oled_StateName(BspOledState_t state) { (void)state; return "ONLINE"; }

static void PressSelect(void)
{
    g_select_press = true;
    TaskMenuUi_Process();
}

static void PressConfirm(void)
{
    g_confirm_press = true;
    TaskMenuUi_Process();
}

int main(void)
{
    uint32_t redraw_count;

    assert(TaskMenuUi_Init());
    assert(TaskMenuUi_GetSelectedTask() == TASK_MENU_TASK_2_LAP_STOP);

    PressSelect();
    assert(TaskMenuUi_GetSelectedTask() == TASK_MENU_TASK_3_BALL_SEQUENCE);
    PressConfirm();
    assert(TaskMenuUi_GetState() == TASK_MENU_STATE_ARMED);
    PressConfirm();
    assert(TaskMenuUi_GetState() == TASK_MENU_STATE_RUNNING);
    assert(strcmp(g_state_text, "RDY") == 0);

    TaskMenuUi_SetRunningElapsedMs(0U, false);
    assert(strcmp(g_state_text, "RDY") == 0);

    TaskMenuUi_SetRunningElapsedMs(1234U, true);
    assert(strcmp(g_state_text, "RUN") == 0);
    assert(g_fill_count > 0U);

    redraw_count = g_clear_count;
    TaskMenuUi_SetRunningElapsedMs(1299U, true);
    assert(g_clear_count == redraw_count);

    TaskMenuUi_SetRunningElapsedMs(1300U, true);
    assert(g_clear_count == redraw_count + 1U);

    TaskMenuUi_SetFinishedResult(true, 2345U);
    assert(TaskMenuUi_GetState() == TASK_MENU_STATE_FINISHED);
    assert(strcmp(g_state_text, "PASS") == 0);
    assert(g_fill_count > 0U);

    puts("task3_oled_timer: PASS");
    return 0;
}
