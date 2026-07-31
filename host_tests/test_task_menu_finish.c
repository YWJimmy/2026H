#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "task_menu_ui.h"
#include "bsp_key.h"

static bool g_select_press;
static bool g_confirm_press;

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

void BSP_Oled_Clear(void) { }
void BSP_Oled_DrawString(uint8_t x, uint8_t page, const char *s)
{
    (void)x; (void)page; (void)s;
}
void BSP_Oled_DrawU32(uint8_t x, uint8_t page, uint32_t value)
{
    (void)x; (void)page; (void)value;
}

int main(void)
{
    TaskMenuTask_t task;
    assert(TaskMenuUi_Init());

    /* Select task 3. */
    g_select_press = true;
    TaskMenuUi_Process();
    assert(TaskMenuUi_GetSelectedTask() == TASK_MENU_TASK_3_BALL_SEQUENCE);

    g_confirm_press = true;
    TaskMenuUi_Process();
    assert(TaskMenuUi_GetState() == TASK_MENU_STATE_ARMED);

    g_confirm_press = true;
    TaskMenuUi_Process();
    assert(TaskMenuUi_TakeStartRequest(&task));
    assert(task == TASK_MENU_TASK_3_BALL_SEQUENCE);

    TaskMenuUi_SetFinishedResult(true, 2345U);
    assert(TaskMenuUi_GetState() == TASK_MENU_STATE_FINISHED);

    /* K0 on DONE must stop final hold before returning to menu. */
    g_confirm_press = true;
    TaskMenuUi_Process();
    assert(TaskMenuUi_GetState() == TASK_MENU_STATE_SELECT);
    assert(TaskMenuUi_TakeStopRequest());

    puts("task_menu_finish: PASS");
    return 0;
}
