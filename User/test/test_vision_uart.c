#include "test_vision_uart.h"

#include "bsp_debug_uart.h"
#include "bsp_vision_uart.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

/*
 * 控制台打印周期，单位为 ms。
 * 设为 0 表示每帧都打印。
 */
#define TEST_VISION_PRINT_PERIOD_MS    0U

/*
 * 心跳打印间隔，单位为 ms。
 * 在未收到任何 K230D 数据时，每隔此时间打印一次心跳，
 * 方便确认 STM32 本身是否正常运行。
 */
#define TEST_VISION_HEARTBEAT_MS       2000U

static bool s_initialized = false;
static bool s_vision_ok = false;
static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;

bool Test_VisionUart_Init(void)
{
    s_initialized = false;
    s_vision_ok = false;
    s_last_print_ms = 0U;
    s_last_heartbeat_ms = 0U;

    /*
     * 第一步：只初始化调试串口，确保至少能打印错误信息。
     */
    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    /*
     * 尽早打印启动横幅，确认调试串口工作正常。
     */
    (void)BSP_Debug_Printf(
        "TEST,VISION_UART,START,PRINT_MS=%lu\r\n",
        (unsigned long)TEST_VISION_PRINT_PERIOD_MS);

    /*
     * 第二步：尝试初始化视觉 UART。
     * 即使失败也不阻塞整体运行，只打印错误并标记。
     */
    if (!BSP_VisionUart_Init())
    {
        (void)BSP_Debug_Printf("ERR,VISION_UART_INIT\r\n");
        /*
         * 不 return false：
         * 主循环仍然运行，通过心跳确认 STM32 存活。
         */
    }
    else
    {
        s_vision_ok = true;
        (void)BSP_Debug_Printf("OK,VISION_UART_INIT\r\n");
    }

    s_last_print_ms = HAL_GetTick();
    s_last_heartbeat_ms = s_last_print_ms;
    s_initialized = true;

    return true;
}

void Test_VisionUart_Update(void)
{
    BspVisionDetection_t det;
    uint32_t now_ms;

    BSP_DebugUart_Process();
    BSP_VisionUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    /*
     * 无论 vision 是否就绪，都周期打印心跳。
     */
    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >=
        TEST_VISION_HEARTBEAT_MS)
    {
        s_last_heartbeat_ms = now_ms;

        (void)BSP_Debug_Printf(
            "VISION,HB,FRAMES=%lu,ERRORS=%lu,VISION_OK=%u\r\n",
            (unsigned long)BSP_VisionUart_GetFrameCount(),
            (unsigned long)BSP_VisionUart_GetErrorCount(),
            s_vision_ok ? 1U : 0U);
    }

    /* vision 未就绪时不再尝试读检测数据 */
    if (!s_vision_ok)
    {
        return;
    }

#if TEST_VISION_PRINT_PERIOD_MS > 0U
    if ((uint32_t)(now_ms - s_last_print_ms) <
        TEST_VISION_PRINT_PERIOD_MS)
    {
        return;
    }
#endif

    if (!BSP_VisionUart_HasNewDetection())
    {
        return;
    }

    det = BSP_VisionUart_GetDetection();

    if (det.has_target)
    {
        (void)BSP_Debug_Printf(
            "SB,1,%u,%u,%u,%u,%u,%u,%u\r\n",
            (unsigned int)det.x1,
            (unsigned int)det.y1,
            (unsigned int)det.x2,
            (unsigned int)det.y2,
            (unsigned int)det.cx,
            (unsigned int)det.cy,
            (unsigned int)det.score_milli);
    }
    else
    {
        (void)BSP_Debug_Printf(
            "SB,0,0,0,0,0,0,0,0\r\n");
    }

#if TEST_VISION_PRINT_PERIOD_MS > 0U
    s_last_print_ms = now_ms;
#endif
}

void Test_VisionUart_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    s_initialized = false;
    s_vision_ok = false;

    (void)BSP_Debug_Printf(
        "TEST,VISION_UART,STOP,"
        "FRAMES=%lu,ERRORS=%lu\r\n",
        (unsigned long)BSP_VisionUart_GetFrameCount(),
        (unsigned long)BSP_VisionUart_GetErrorCount());
}

bool Test_VisionUart_IsInitialized(void)
{
    return s_initialized;
}
