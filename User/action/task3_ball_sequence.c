#include "task3_ball_sequence.h"

#include "bsp_debug_uart.h"
#include "test_ball_balance.h"

#include <stddef.h>

/*
 * Adapter that runs the proven 54e87bc Task 3 controller under the frame
 * MainApp/AppTaskManager interface. Task 2 and Task 4 remain independent.
 */
static bool s_initialized = false;
static bool s_started = false;
static bool s_user_stopped = false;
static bool s_natural_finished = false;
static bool s_passed = false;
static bool s_finish_reported = false;
static uint32_t s_fault_detail = 0U;

bool Task3BallSequence_Init(void)
{
    s_initialized = true;
    s_started = false;
    s_user_stopped = false;
    s_natural_finished = false;
    s_passed = false;
    s_finish_reported = false;
    s_fault_detail = 0U;
    return true;
}

bool Task3BallSequence_Start(uint32_t start_timestamp_ms)
{
    (void)start_timestamp_ms;

    if ((!s_initialized) || s_started)
    {
        s_fault_detail = 20U;
        return false;
    }

    if (!Test_BallBalance_Init())
    {
        s_fault_detail = 101U;
        return false;
    }

    s_started = true;
    s_user_stopped = false;
    s_natural_finished = false;
    s_passed = false;
    s_finish_reported = false;
    s_fault_detail = 0U;

    (void)BSP_Debug_Printf(
        "T3,START=1,CTRL=54E_ONE_SHOT_ANTI_DRIFT,"
        "SEQ=O_TO_POS5_TO_NEG5\r\n");
    return true;
}

Task3BallSequenceResult_t Task3BallSequence_Update(uint32_t now_ms)
{
    (void)now_ms;

    if ((!s_initialized) || (!s_started))
    {
        s_fault_detail = 1U;
        return TASK3_BALL_SEQUENCE_RESULT_FAULT;
    }

    if (s_user_stopped)
    {
        return TASK3_BALL_SEQUENCE_RESULT_FINISHED;
    }

    Test_BallBalance_Update();

    if (Test_BallBalance_IsFinished())
    {
        s_natural_finished = true;
        s_passed = Test_BallBalance_Passed();

        if (!s_finish_reported)
        {
            s_finish_reported = true;
            (void)BSP_Debug_Printf(
                "T3,INTEGRATED_FINISH=1,RESULT=%s,MS=%lu,"
                "HOLD=ANTI_DRIFT\r\n",
                s_passed ? "PASS" : "FAIL",
                (unsigned long)Test_BallBalance_GetElapsedMs());
        }

        return TASK3_BALL_SEQUENCE_RESULT_FINISHED;
    }

    return TASK3_BALL_SEQUENCE_RESULT_RUNNING;
}

bool Task3BallSequence_Maintain(uint32_t now_ms)
{
    (void)now_ms;

    if (!s_started)
    {
        return true;
    }

    if (s_user_stopped)
    {
        return true;
    }

    /* Keep the 54e FINISHED_ANTI_DRIFT loop alive after PASS is latched. */
    Test_BallBalance_Update();
    return Test_BallBalance_IsInitialized();
}

bool Task3BallSequence_RequestStop(void)
{
    if ((!s_initialized) || (!s_started))
    {
        return false;
    }

    /* Natural completion keeps the servo enabled for anti-drift maintain. */
    if (s_natural_finished)
    {
        return true;
    }

    Test_BallBalance_Stop();
    s_user_stopped = true;
    return true;
}

bool Task3BallSequence_IsStopped(void)
{
    return s_user_stopped || s_natural_finished || (!s_started);
}

void Task3BallSequence_ForceSafeStop(void)
{
    if (Test_BallBalance_IsInitialized())
    {
        Test_BallBalance_Stop();
    }

    s_started = false;
    s_user_stopped = true;
    s_natural_finished = false;
    s_passed = false;
    s_finish_reported = false;
}

bool Task3BallSequence_GetElapsedMs(
    uint32_t now_ms,
    uint32_t *elapsed_ms)
{
    (void)now_ms;

    if ((!s_started) || (elapsed_ms == NULL))
    {
        return false;
    }

    *elapsed_ms = Test_BallBalance_GetElapsedMs();
    return true;
}

const char *Task3BallSequence_GetPhaseText(void)
{
    if (!s_started)
    {
        return "T3 READY";
    }

    if (s_user_stopped)
    {
        return "T3 STOPPED";
    }

    if (s_natural_finished)
    {
        return s_passed ? "T3 PASS HOLD" : "T3 FAIL HOLD";
    }

    return "T3 O +5 -5";
}

uint32_t Task3BallSequence_GetFaultDetail(void)
{
    return s_fault_detail;
}
