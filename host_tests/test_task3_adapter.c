#include "task3_ball_sequence.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

static int g_init_ok = 1;
static int g_initialized = 0;
static int g_finished = 0;
static int g_passed = 0;
static int g_timer = 0;
static unsigned long g_elapsed = 0;
static int g_updates = 0;
static int g_stops = 0;

int BSP_Debug_Printf(const char *format, ...)
{
    (void)format;
    return 1;
}

bool Test_BallBalance_Init(void)
{
    g_initialized = g_init_ok;
    g_finished = 0;
    g_timer = 0;
    g_elapsed = 0;
    return g_init_ok != 0;
}
void Test_BallBalance_Update(void)
{
    g_updates++;
    if (g_initialized && !g_finished)
    {
        g_timer = 1;
        g_elapsed += 100;
    }
}
void Test_BallBalance_Stop(void) { g_initialized = 0; g_stops++; }
bool Test_BallBalance_IsInitialized(void) { return g_initialized != 0; }
bool Test_BallBalance_IsFinished(void) { return g_finished != 0; }
bool Test_BallBalance_Passed(void) { return g_finished && g_passed; }
bool Test_BallBalance_IsTimerRunning(void) { return g_timer && !g_finished; }
uint32_t Test_BallBalance_GetElapsedMs(void) { return (uint32_t)g_elapsed; }

int main(void)
{
    uint32_t elapsed = 0;
    assert(Task3BallSequence_Init());
    assert(Task3BallSequence_Start(123U));
    assert(Task3BallSequence_Update(200U) == TASK3_BALL_SEQUENCE_RESULT_RUNNING);
    assert(Task3BallSequence_GetElapsedMs(200U, &elapsed));
    assert(elapsed == 100U);
    assert(Task3BallSequence_GetPhaseText()[0] == 'T');

    g_finished = 1;
    g_passed = 1;
    assert(Task3BallSequence_Update(300U) == TASK3_BALL_SEQUENCE_RESULT_FINISHED);
    assert(Task3BallSequence_IsStopped());
    assert(Task3BallSequence_RequestStop());
    assert(g_stops == 0); /* natural finish keeps anti-drift active */
    assert(Task3BallSequence_Maintain(400U));
    assert(g_updates >= 3);

    Task3BallSequence_ForceSafeStop();
    assert(g_stops == 1);

    assert(Task3BallSequence_Init());
    assert(Task3BallSequence_Start(500U));
    assert(Task3BallSequence_RequestStop());
    assert(Task3BallSequence_IsStopped());
    assert(g_stops == 2);

    puts("task3_adapter: PASS");
    return 0;
}
