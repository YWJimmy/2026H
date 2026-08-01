#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "test_ball_balance.h"
#include "vision.h"

static uint32_t g_tick_ms;
static uint16_t g_servo_pulse;
static bool g_servo_enabled;
static VisionStatus_t g_vision;

uint32_t HAL_GetTick(void) { return g_tick_ms; }
bool BSP_DebugUart_Init(void) { return true; }
void BSP_DebugUart_Process(void) { }
int BSP_Debug_Printf(const char *fmt, ...) { (void)fmt; return 0; }
bool BSP_Servo_Init(void) { g_servo_pulse = 1500U; return true; }
bool BSP_Servo_Enable(void) { g_servo_enabled = true; return true; }
void BSP_Servo_Disable(void) { g_servo_enabled = false; }
bool BSP_Servo_IsEnabled(void) { return g_servo_enabled; }
bool BSP_Servo_SetPulseUs(uint16_t pulse) { g_servo_pulse = pulse; return true; }
uint16_t BSP_Servo_GetPulseUs(void) { return g_servo_pulse; }
bool Vision_Init(void) { memset(&g_vision, 0, sizeof(g_vision)); g_vision.initialized = true; return true; }
void Vision_Update(void) { }
void Vision_Stop(void) { g_vision.initialized = false; }
bool Vision_GetStatus(VisionStatus_t *status) { *status = g_vision; return true; }

static int16_t CenterXToMm(uint16_t cx)
{
    if (cx <= 153U) return -120;
    if (cx <= 447U)
    {
        return (int16_t)(-120 + ((int32_t)(cx - 153U) * 70 + 147) / 294);
    }
    if (cx <= 653U)
    {
        return (int16_t)(-50 + ((int32_t)(cx - 447U) * 50 + 103) / 206);
    }
    if (cx <= 869U)
    {
        return (int16_t)(((int32_t)(cx - 653U) * 50 + 108) / 216);
    }
    if (cx <= 1160U)
    {
        return (int16_t)(50 + ((int32_t)(cx - 869U) * 70 + 145) / 291);
    }
    return 120;
}

static void Feed(uint16_t cx, uint32_t dt_ms)
{
    g_tick_ms += dt_ms;
    g_vision.sequence++;
    g_vision.has_frame = true;
    g_vision.data_valid = true;
    g_vision.frame.found = true;
    g_vision.frame.center_x = cx;
    g_vision.frame.physical_x_mm = CenterXToMm(cx);
    g_vision.valid_frame_count++;
    Test_BallBalance_Update();
}

static void StartAtCenter(void)
{
    g_tick_ms = 100U;
    g_servo_pulse = 0U;
    g_servo_enabled = false;
    memset(&g_vision, 0, sizeof(g_vision));

    assert(Test_BallBalance_Init());
    assert(g_servo_enabled);
    /* Task 3 starts toward +5 cm immediately; O is the prescribed start. */
    assert(g_servo_pulse == 1775U);
}

static void TestOneShotPass(void)
{
    uint32_t neg5_arrival_ms;

    StartAtCenter();

    Feed(680U, 50U);
    Feed(720U, 50U);
    Feed(760U, 50U);
    Feed(800U, 50U);
    Feed(840U, 50U);
    assert(g_servo_pulse == 1420U);

    /* The reversal must stay strong while rightward velocity remains. */
    Feed(865U, 50U);
    assert(g_servo_pulse == 1420U);
    Feed(870U, 50U);
    Feed(870U, 50U);
    Feed(865U, 50U);
    Feed(855U, 50U);
    assert(g_servo_pulse <= 1550U);

    Feed(820U, 50U);
    Feed(760U, 50U);
    Feed(690U, 50U);
    Feed(620U, 50U);
    Feed(555U, 50U);
    Feed(510U, 50U);
    Feed(472U, 50U);
    neg5_arrival_ms = g_tick_ms - 100U;

    /* Near -5 cm with leftward speed: brake right, never finish while moving. */
    Feed(455U, 50U);
    assert(g_servo_pulse > 1675U);
    Feed(447U, 50U);
    assert(g_servo_pulse > 1675U);
    assert(!Test_BallBalance_IsFinished());

    /* +/-1 cm is acceptable. Finish on the first truly stopped frame. */
    Feed(488U, 50U);
    assert(!Test_BallBalance_IsFinished());
    while (!Test_BallBalance_IsFinished())
    {
        Feed(488U, 50U);
    }

    assert(Test_BallBalance_IsFinished());
    assert(Test_BallBalance_Passed());
    assert(g_servo_pulse <= 1675U);
    assert(g_tick_ms - 100U < 5000U);
    /* Timer starts with the command and freezes on first arrival at -5 cm. */
    assert(Test_BallBalance_GetElapsedMs() == neg5_arrival_ms);

    /*
     * PASS time is frozen, but post-finish control must oppose a later drift
     * toward -3 cm instead of leaving the servo at a fixed neutral pulse.
     */
    {
        uint32_t latched_ms = Test_BallBalance_GetElapsedMs();
        Feed(510U, 50U);
        assert(Test_BallBalance_IsFinished());
        assert(Test_BallBalance_Passed());
        assert(Test_BallBalance_GetElapsedMs() == latched_ms);
        assert(g_servo_pulse < 1675U);

        Feed(530U, 50U);
        assert(Test_BallBalance_GetElapsedMs() == latched_ms);
        assert(g_servo_pulse < 1675U);
    }

    Test_BallBalance_Stop();
    assert(!g_servo_enabled);
}

static void TestBudgetOverrunKeepsControl(void)
{
    StartAtCenter();

    for (int i = 0; i < 110; ++i)
    {
        Feed(653U, 50U);
    }

    assert(!Test_BallBalance_IsFinished());
    assert(!Test_BallBalance_Passed());
    assert(Test_BallBalance_IsTimerRunning());
    assert(Test_BallBalance_GetElapsedMs() > 5000U);
    assert(g_servo_pulse == 1775U);
    Test_BallBalance_Stop();
}

int main(void)
{
    TestOneShotPass();
    TestBudgetOverrunKeepsControl();
    puts("task3_optimized: PASS");
    return 0;
}
