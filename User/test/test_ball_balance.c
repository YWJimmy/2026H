#include "test_ball_balance.h"

#include "ball_balance_control.h"
#include "ball_balance_control_config.h"
#include "bsp_debug_uart.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "stm32f4xx_hal.h"
#include "task_menu_ui.h"
#include "vision.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/*
 * ---------------------------------------------------------------------------
 * Local strategy / state definitions
 * ---------------------------------------------------------------------------
 */
typedef enum
{
    BALL_STRATEGY_AB_HOLD = 0,
    BALL_STRATEGY_3PHASE,
    BALL_STRATEGY_COUNT
} BallTestStrategy_t;

typedef enum
{
    BALL_TEST_STATE_MENU = 0,
    BALL_TEST_STATE_RUNNING
} BallTestState_t;

/*
 * ---------------------------------------------------------------------------
 * Timing constants
 * ---------------------------------------------------------------------------
 */
#define TEST_BALL_BALANCE_PRINT_PERIOD_MS     ((uint32_t)50U)
#define TEST_BALL_BALANCE_HEARTBEAT_MS        ((uint32_t)2000U)
#define TEST_BALL_BALANCE_OLED_REFRESH_MS     ((uint32_t)200U)

/*
 * ---------------------------------------------------------------------------
 * Global state
 * ---------------------------------------------------------------------------
 */
static BallTestStrategy_t s_strategy =
    BALL_STRATEGY_AB_HOLD;
static BallTestState_t s_state =
    BALL_TEST_STATE_MENU;
static bool s_initialized = false;
static bool s_vision_ok = false;

/*
 * ---------------------------------------------------------------------------
 * Timing trackers
 * ---------------------------------------------------------------------------
 */
static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t s_last_oled_ms = 0U;

/*
 * ---------------------------------------------------------------------------
 * Event / error tracking
 * ---------------------------------------------------------------------------
 */
static uint32_t s_last_event_sequence = 0U;
static uint32_t s_last_servo_error_count = 0U;

/*
 * ---------------------------------------------------------------------------
 * A-B hold cycle state
 * ---------------------------------------------------------------------------
 */
static uint32_t s_target_hold_start_ms = 0U;
static bool s_target_hold_active = false;

/*
 * ---------------------------------------------------------------------------
 * 3-phase cycle state
 * ---------------------------------------------------------------------------
 */
static uint8_t s_cycle_phase = 0U;
static uint32_t s_cycle_start_ms = 0U;
static const uint16_t s_cycle_targets[3] =
    {
        (uint16_t)BALL_BALANCE_TARGET_CX_0,
        (uint16_t)BALL_BALANCE_TARGET_CX_P5,
        (uint16_t)BALL_BALANCE_TARGET_CX_N5};
static const uint8_t s_cycle_hold[3] = {1U, 0U, 1U};
/*
 * Per-phase brake distance gain.
 * Phase 0: approach 0cm   — gain 10 (231 px approach from -5cm)
 * Phase 1: approach +5cm  — gain 10 (216 px approach from  0cm)
 * Phase 2: approach -5cm  — gain 15 (422 px long approach)
 */
static const int32_t s_cycle_brake_gains[3] =
    {
        (int32_t)10,                            /* 10: Phase 0 */
        (int32_t)10,                            /* 10: Phase 1 bounce approach */
        (int32_t)15                             /* 15: Phase 2 long approach */
    };

/*
 * ---------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------------
 */
static int32_t Test_BallBalance_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static const char *BallStrategy_Name(
    BallTestStrategy_t strategy)
{
    switch (strategy)
    {
        case BALL_STRATEGY_AB_HOLD:
            return "AB HOLD";

        case BALL_STRATEGY_3PHASE:
            return "3PHASE";

        default:
            return "?";
    }
}

/*
 * Map ball strategy to the corresponding menu task.
 */
static TaskMenuTask_t BallStrategy_ToMenuTask(
    BallTestStrategy_t strategy)
{
    switch (strategy)
    {
        case BALL_STRATEGY_AB_HOLD:
            return TASK_MENU_TASK_4_AB_HOLD;

        case BALL_STRATEGY_3PHASE:
        default:
            return TASK_MENU_TASK_3_BALL_SEQUENCE;
    }
}

/*
 * ---------------------------------------------------------------------------
 * OLED running screen
 * ---------------------------------------------------------------------------
 */
static void BallTest_RenderRunning(void)
{
    BallBalanceControlStatus_t control;
    char line[32]; /* OLED 128px / 6 = 21 chars, 32 for safety */

    if (!BallBalanceControl_GetStatus(&control))
    {
        return;
    }

    BSP_Oled_Clear();

    /* Page 0: strategy + phase (3-phase only) */
    if (s_strategy == BALL_STRATEGY_3PHASE)
    {
        (void)snprintf(line, sizeof(line),
            "3PH P%u %s",
            (unsigned int)s_cycle_phase,
            (s_cycle_hold[s_cycle_phase] != 0U)
                ? "HOLD"
                : "BOUNCE");
    }
    else
    {
        (void)snprintf(line, sizeof(line),
            "A-B HOLD");
    }
    BSP_Oled_DrawString(0U, 0U, line);

    /* Page 2: CX + TARGET */
    (void)snprintf(line, sizeof(line),
        "CX=%-5ld T=%-5ld",
        (long)control.center_x,
        (long)control.target_x);
    BSP_Oled_DrawString(0U, 2U, line);

    /* Page 4: ERR + SPD + MODE */
    (void)snprintf(line, sizeof(line),
        "E=%-4ld S=%-4ld %s",
        (long)control.error,
        (long)control.speed,
        BallBalanceControl_ModeName(control.mode));
    BSP_Oled_DrawString(0U, 4U, line);

    /* Page 6: PULSE + stop hint */
    (void)snprintf(line, sizeof(line),
        "PLS=%-4u K0:STOP",
        (unsigned int)control.servo_pulse_us);
    BSP_Oled_DrawString(0U, 6U, line);
}

/*
 * ---------------------------------------------------------------------------
 * A-B hold cycle (original HEAD logic, renamed)
 * ---------------------------------------------------------------------------
 */
static bool Test_BallBalance_UpdateTargetCycle_AB(
    const BallBalanceControlStatus_t *control,
    uint32_t now_ms)
{
    int32_t next_target;

    if ((control == NULL) ||
        !control->has_target ||
        !control->vision_data_valid ||
        (Test_BallBalance_AbsI32(control->error) >
         BALL_BALANCE_DEADZONE_PX))
    {
        s_target_hold_active = false;
        return true;
    }

    if (!s_target_hold_active)
    {
        s_target_hold_start_ms = now_ms;
        s_target_hold_active = true;
        return true;
    }

    if ((uint32_t)(now_ms - s_target_hold_start_ms) <
        BALL_BALANCE_TARGET_HOLD_MS)
    {
        return true;
    }

    next_target = (control->target_x == BALL_BALANCE_TARGET_CX_A)
        ? BALL_BALANCE_TARGET_CX_B
        : BALL_BALANCE_TARGET_CX_A;

    s_target_hold_active = false;

    if (!BallBalanceControl_SetTargetX(next_target))
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,CYCLE,NEW_TARGET=%ld\r\n",
        (long)next_target);
    return true;
}

/*
 * ---------------------------------------------------------------------------
 * 3-phase cycle (from 7a35ce0, adapted to BallBalanceControl module)
 *
 * Phase 0: 0cm  (653), hold 5s  -> Phase 1
 * Phase 1: +5cm (869), bounce    -> Phase 2  (immediate switch)
 * Phase 2: -5cm (447), hold 5s  -> Phase 0
 * ---------------------------------------------------------------------------
 */
static bool Test_BallBalance_UpdateTargetCycle_3Phase(
    const BallBalanceControlStatus_t *control,
    uint32_t now_ms)
{
    bool do_switch;

    if ((control == NULL) ||
        !control->has_target ||
        !control->vision_data_valid)
    {
        s_cycle_start_ms = 0U;
        return true;
    }

    /*
     * Only cycle when the ball is inside the dead zone.
     */
    if (Test_BallBalance_AbsI32(control->error) >
        BALL_BALANCE_DEADZONE_PX)
    {
        s_cycle_start_ms = 0U;
        return true;
    }

    do_switch = false;

    if (s_cycle_hold[s_cycle_phase] == 0U)
    {
        /* Bounce phase: switch immediately. */
        do_switch = true;
    }
    else
    {
        /* Hold phase: wait BALL_BALANCE_CYCLE_HOLD_MS. */
        if (s_cycle_start_ms == 0U)
        {
            s_cycle_start_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s_cycle_start_ms) >=
                 BALL_BALANCE_CYCLE_HOLD_MS)
        {
            do_switch = true;
        }
    }

    if (!do_switch)
    {
        return true;
    }

    /* Advance to next phase. */
    s_cycle_start_ms = 0U;
    s_cycle_phase = (s_cycle_phase + 1U) % 3U;

    /*
     * Set per-phase brake gain.
     * Phase 0: 5, Phase 1: 8, Phase 2: 12.
     */
    (void)BallBalanceControl_SetBrakeGain(
        s_cycle_brake_gains[s_cycle_phase]);

    if (!BallBalanceControl_SetTargetX(
            (int32_t)s_cycle_targets[s_cycle_phase]))
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,CYCLE,PHASE=%u,HOLD=%u,TARGET=%u,"
        "BRAKE_GAIN=%ld\r\n",
        (unsigned int)s_cycle_phase,
        (unsigned int)s_cycle_hold[s_cycle_phase],
        (unsigned int)s_cycle_targets[s_cycle_phase],
        (long)BallBalanceControl_GetBrakeGain());
    return true;
}

/*
 * ---------------------------------------------------------------------------
 * Strategy dispatcher
 * ---------------------------------------------------------------------------
 */
static bool Test_BallBalance_UpdateTargetCycle(
    const BallBalanceControlStatus_t *control,
    uint32_t now_ms)
{
    if (s_strategy == BALL_STRATEGY_3PHASE)
    {
        return Test_BallBalance_UpdateTargetCycle_3Phase(
            control,
            now_ms);
    }

    return Test_BallBalance_UpdateTargetCycle_AB(
        control,
        now_ms);
}

/*
 * ---------------------------------------------------------------------------
 * Event printer (unchanged from HEAD)
 * ---------------------------------------------------------------------------
 */
static void Test_BallBalance_PrintEvent(
    const BallBalanceControlStatus_t *control)
{
    if ((control == NULL) ||
        (control->event_sequence == s_last_event_sequence))
    {
        return;
    }

    s_last_event_sequence = control->event_sequence;

    (void)BSP_Debug_Printf(
        "BALL_BAL,EVENT=%s,PULSE=%u,SEQ=%lu\r\n",
        BallBalanceControl_EventName(control->last_event),
        (unsigned int)control->servo_pulse_us,
        (unsigned long)control->event_sequence);
}

/*
 * ---------------------------------------------------------------------------
 * Reset cycle state for a fresh strategy start
 * ---------------------------------------------------------------------------
 */
static void BallTest_ResetStrategy(void)
{
    s_target_hold_active = false;
    s_target_hold_start_ms = 0U;

    s_cycle_phase = 0U;
    s_cycle_start_ms = 0U;

    /* Default brake gain. */
    (void)BallBalanceControl_SetBrakeGain(
        BALL_BALANCE_BRAKE_DISTANCE_GAIN);
}

/*
 * ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------
 */
bool Test_BallBalance_Init(void)
{
    BallBalanceControlStatus_t control;
    uint32_t now_ms;

    s_initialized = false;
    s_vision_ok = false;
    s_strategy = BALL_STRATEGY_AB_HOLD;
    s_state = BALL_TEST_STATE_MENU;
    s_last_event_sequence = 0U;
    s_last_servo_error_count = 0U;
    s_last_print_ms = 0U;
    s_last_heartbeat_ms = 0U;
    s_last_oled_ms = 0U;

    BallTest_ResetStrategy();

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,START,ARCH=BSP_MODULE,"
        "TARGETS=%ld/%ld,HOLD_MS=%lu,DEADZONE=%ld,KP_Q10=%ld,"
        "BRAKE_GAIN=%ld,BRAKE_BASE=%ld,BRAKE_MAX=%ld,"
        "LIMS=PUSH_%u_%u_BRAKE_%u_%u,"
        "STUCK=%ld/%lu,"
        "3PHASE_T0=%ld,TP5=%ld,TN5=%ld,"
        "BRAKE_P0=%ld,P1=%ld,P2=%ld\r\n",
        (long)BALL_BALANCE_TARGET_CX_A,
        (long)BALL_BALANCE_TARGET_CX_B,
        (unsigned long)BALL_BALANCE_TARGET_HOLD_MS,
        (long)BALL_BALANCE_DEADZONE_PX,
        (long)BALL_BALANCE_PUSH_KP_Q10,
        (long)BALL_BALANCE_BRAKE_DISTANCE_GAIN,
        (long)BALL_BALANCE_BRAKE_BASE_US,
        (long)BALL_BALANCE_BRAKE_MAX_US,
        (unsigned int)BALL_BALANCE_SERVO_PUSH_MIN_US,
        (unsigned int)BALL_BALANCE_SERVO_PUSH_MAX_US,
        (unsigned int)BALL_BALANCE_SERVO_BRAKE_MIN_US,
        (unsigned int)BALL_BALANCE_SERVO_BRAKE_MAX_US,
        (long)BALL_BALANCE_STUCK_SPEED_PX,
        (unsigned long)BALL_BALANCE_STUCK_BOOST_US,
        (long)BALL_BALANCE_TARGET_CX_0,
        (long)BALL_BALANCE_TARGET_CX_P5,
        (long)BALL_BALANCE_TARGET_CX_N5,
        (long)s_cycle_brake_gains[0],
        (long)s_cycle_brake_gains[1],
        (long)s_cycle_brake_gains[2]);

    /*
     * Init vision first (non-fatal) so the ball control module
     * can receive frames once started.
     */
    if (!Vision_Init())
    {
        (void)BSP_Debug_Printf("ERR,VISION_INIT\r\n");
    }
    else
    {
        s_vision_ok = true;
        (void)BSP_Debug_Printf("OK,VISION_INIT\r\n");
    }

    if (!BallBalanceControl_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,BALL_BALANCE_CONTROL_INIT\r\n");
        return false;
    }

    if (BallBalanceControl_GetStatus(&control))
    {
        (void)BSP_Debug_Printf(
            "OK,BALL_CONTROL,PULSE=%u,EN=%u,MODE=%s\r\n",
            (unsigned int)control.servo_pulse_us,
            control.servo_enabled ? 1U : 0U,
            BallBalanceControl_ModeName(control.mode));
    }

    /* Init keys and OLED for menu interaction. */
    if (!BSP_Key_Init())
    {
        (void)BSP_Debug_Printf("ERR,KEY_INIT\r\n");
        return false;
    }

    if (!BSP_Oled_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,OLED_INIT,CHECK=I2C1_400KHZ_IRQ\r\n");
        return false;
    }

    if (!TaskMenuUi_Init())
    {
        (void)BSP_Debug_Printf("ERR,TASK_MENU_INIT\r\n");
        return false;
    }

    now_ms = HAL_GetTick();
    s_last_print_ms = now_ms;
    s_last_heartbeat_ms = now_ms;
    s_last_oled_ms = now_ms;
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "BALL_BAL,READY,STATE=MENU,"
        "KEY=UP_PA0_K0_PE4,OLED=SSD1306_128X64\r\n");
    return true;
}

void Test_BallBalance_Update(void)
{
    VisionStatus_t vision_status;
    BallBalanceControlStatus_t control;
    bool have_vision_status = false;
    bool have_control_status;
    bool control_update_ok = true;
    uint32_t now_ms;
    TaskMenuTask_t start_task;

    if (!s_initialized)
    {
        return;
    }

    BSP_DebugUart_Process();
    BSP_Key_Process();
    BSP_Oled_Process();

    switch (s_state)
    {
        case BALL_TEST_STATE_MENU:
            TaskMenuUi_Process();

            if (TaskMenuUi_TakeStartRequest(&start_task))
            {
                /*
                 * Map menu task to ball strategy.
                 */
                switch (start_task)
                {
                    case TASK_MENU_TASK_3_BALL_SEQUENCE:
                        s_strategy = BALL_STRATEGY_3PHASE;
                        break;

                    case TASK_MENU_TASK_4_AB_HOLD:
                        s_strategy = BALL_STRATEGY_AB_HOLD;
                        break;

                    default:
                        /*
                         * Unknown task: log and return to
                         * menu immediately.
                         */
                        (void)BSP_Debug_Printf(
                            "BALL_BAL,NO_IMPL,TASK=%u\r\n",
                            (unsigned int)start_task);
                        TaskMenuUi_SetFinished();
                        return;
                }

                BallTest_ResetStrategy();

                /*
                 * Set initial target for the selected strategy.
                 */
                if (s_strategy == BALL_STRATEGY_3PHASE)
                {
                    (void)BallBalanceControl_SetTargetX(
                        (int32_t)s_cycle_targets[0]);
                }
                else
                {
                    (void)BallBalanceControl_SetTargetX(
                        BALL_BALANCE_TARGET_CX_A);
                }

                s_state = BALL_TEST_STATE_RUNNING;
                s_last_oled_ms = 0U; /* force immediate redraw */
                BallTest_RenderRunning();

                (void)BSP_Debug_Printf(
                    "BALL_BAL,START,STRATEGY=%s\r\n",
                    BallStrategy_Name(s_strategy));
            }
            break;

        case BALL_TEST_STATE_RUNNING:
            /*
             * Run vision and ball control.
             */
            if (s_vision_ok)
            {
                Vision_Update();
                have_vision_status =
                    Vision_GetStatus(&vision_status);

                if (have_vision_status)
                {
                    control_update_ok =
                        BallBalanceControl_Update(
                            &vision_status);
                }
            }

            have_control_status =
                BallBalanceControl_GetStatus(&control);
            now_ms = HAL_GetTick();

            if (have_control_status)
            {
                if (!Test_BallBalance_UpdateTargetCycle(
                        &control,
                        now_ms))
                {
                    (void)BSP_Debug_Printf(
                        "ERR,BALL_TARGET_SWITCH\r\n");
                }
                else
                {
                    /*
                     * Refresh status after a possible
                     * cycle switch.
                     */
                    have_control_status =
                        BallBalanceControl_GetStatus(
                            &control);
                }

                Test_BallBalance_PrintEvent(&control);
            }

            /*
             * Servo error reporting (HEAD logic).
             */
            if (!control_update_ok &&
                have_control_status &&
                (control.servo_error_count !=
                 s_last_servo_error_count))
            {
                s_last_servo_error_count =
                    control.servo_error_count;

                (void)BSP_Debug_Printf(
                    "ERR,BALL_SERVO_CMD,COUNT=%lu,"
                    "MODE=%s,EVENT=%s\r\n",
                    (unsigned long)control.servo_error_count,
                    BallBalanceControl_ModeName(control.mode),
                    BallBalanceControl_EventName(
                        control.last_event));
            }
            else if (!control_update_ok &&
                     !have_control_status)
            {
                /*
                 * Control status unavailable fallback
                 * (HEAD logic).
                 */
                (void)BSP_Debug_Printf(
                    "ERR,BALL_CONTROL_UPDATE,"
                    "STATUS=UNAVAILABLE\r\n");
            }

            /*
             * Heartbeat log.
             */
            if ((uint32_t)(now_ms - s_last_heartbeat_ms) >=
                TEST_BALL_BALANCE_HEARTBEAT_MS)
            {
                s_last_heartbeat_ms = now_ms;

                if (s_vision_ok &&
                    have_vision_status &&
                    have_control_status)
                {
                    (void)BSP_Debug_Printf(
                        "BALL_BAL,HB,SEQ=%lu,VF=%lu,IF=%lu,"
                        "PO=%lu,UE=%lu,VALID=%u,MODE=%s\r\n",
                        (unsigned long)vision_status.sequence,
                        (unsigned long)vision_status.valid_frame_count,
                        (unsigned long)vision_status.invalid_frame_count,
                        (unsigned long)vision_status.protocol_overflow_count,
                        (unsigned long)vision_status.uart_error_count,
                        vision_status.data_valid ? 1U : 0U,
                        BallBalanceControl_ModeName(control.mode));
                }
                else if (have_control_status)
                {
                    (void)BSP_Debug_Printf(
                        "BALL_BAL,HB,VISION=0,PULSE=%u,"
                        "MODE=%s\r\n",
                        (unsigned int)control.servo_pulse_us,
                        BallBalanceControl_ModeName(control.mode));
                }
            }

            /*
             * Period status log.
             */
            if (have_control_status &&
                ((uint32_t)(now_ms - s_last_print_ms) >=
                 TEST_BALL_BALANCE_PRINT_PERIOD_MS))
            {
                s_last_print_ms = now_ms;

                if (control.has_target)
                {
                    (void)BSP_Debug_Printf(
                        "BALL_BAL,CX=%ld,TARGET=%ld,ERR=%ld,"
                        "SPD=%ld,D=%ld,"
                        "MODE=%s,BOOST=%lu,PULSE=%u,"
                        "BRAKE_G=%ld,PH=%u\r\n",
                        (long)control.center_x,
                        (long)control.target_x,
                        (long)control.error,
                        (long)control.speed,
                        (long)control.delta_us,
                        BallBalanceControl_ModeName(control.mode),
                        (unsigned long)control.stuck_boost_us,
                        (unsigned int)control.servo_pulse_us,
                        (long)control.brake_distance_gain,
                        (unsigned int)s_cycle_phase);
                }
                else
                {
                    (void)BSP_Debug_Printf(
                        "BALL_BAL,NO_TARGET,LOST=%lu,"
                        "MODE=%s,PULSE=%u\r\n",
                        (unsigned long)control.lost_frames,
                        BallBalanceControl_ModeName(control.mode),
                        (unsigned int)control.servo_pulse_us);
                }
            }

            /*
             * OLED refresh at reduced rate.
             */
            if ((uint32_t)(now_ms - s_last_oled_ms) >=
                TEST_BALL_BALANCE_OLED_REFRESH_MS)
            {
                s_last_oled_ms = now_ms;
                BallTest_RenderRunning();
            }

            /*
             * KEY0 stops the running task.
             */
            if (BSP_Key_TakePress(BSP_KEY_CONFIRM))
            {
                (void)BSP_Debug_Printf(
                    "BALL_BAL,STOP_REQUEST,STRATEGY=%s\r\n",
                    BallStrategy_Name(s_strategy));

                BallBalanceControl_Stop();
                s_state = BALL_TEST_STATE_MENU;
                TaskMenuUi_SetFinished();

                /*
                 * Re-init the control module so the servo
                 * returns to center and is ready for the
                 * next start.
                 */
                (void)BallBalanceControl_Init();
            }
            break;

        default:
            s_state = BALL_TEST_STATE_MENU;
            break;
    }
}

void Test_BallBalance_Stop(void)
{
    BallBalanceControlStatus_t control;
    uint16_t pulse_us =
        BALL_BALANCE_SERVO_CENTER_US;

    if (!s_initialized)
    {
        return;
    }

    if (s_vision_ok)
    {
        Vision_Stop();
    }

    if (BallBalanceControl_GetStatus(&control))
    {
        pulse_us = control.servo_pulse_us;
    }

    BallBalanceControl_Stop();

    BSP_Oled_Clear();

    s_vision_ok = false;
    s_initialized = false;

    (void)BSP_Debug_Printf(
        "BALL_BAL,STOP,PREV_PULSE=%u,CENTER=%u\r\n",
        (unsigned int)pulse_us,
        (unsigned int)BALL_BALANCE_SERVO_CENTER_US);
}

bool Test_BallBalance_IsInitialized(void)
{
    return s_initialized;
}
