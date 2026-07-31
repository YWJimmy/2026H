#include "test_ball_balance.h"

#include "bsp_debug_uart.h"
#include "bsp_servo.h"
#include "bsp_vision_uart.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdlib.h>

/*
 * Target ball center X in 1280x960 coordinates.
 * Cyclic: hold at each target for HOLD_MS, then switch.
 */
#define BALL_TARGET_CX_A        447
#define BALL_TARGET_CX_B        869
#define BALL_CYCLE_HOLD_MS      5000U

/*
 * Servo pulse width limits (us).
 * 1700 = stable/level.
 */
#define SERVO_CENTER_US         1700U
#define SERVO_PUSH_MIN_US       1300U
#define SERVO_PUSH_MAX_US       2000U
#define SERVO_BRAKE_MIN_US      1300U
#define SERVO_BRAKE_MAX_US      2000U

/*
 * P-gain: push strength when ball is not braking.
 */
#define BALL_PUSH_KP            1.0f

/*
 * Braking: start braking when distance < |speed| * BRAKE_GAIN.
 * Higher = brake earlier.
 */
#define BALL_BRAKE_GAIN         5.0f

/*
 * Base brake pulse offset from center (us).
 * Actual brake = BASE + |speed| * SPEED_FACTOR.
 */
#define BALL_BRAKE_BASE_US      200U
#define BALL_BRAKE_SPEED_FACTOR 1.5f

/*
 * Maximum brake pulse offset (us).
 */
/*
 * Stuck detection: wait N frames, then apply one large jump.
 * Normal push is gentle; big kick only when truly stuck.
 */
#define BALL_STUCK_SPEED_PX     3
#define BALL_STUCK_WAIT         15
#define BALL_STUCK_BOOST_US     100U
#define BALL_STUCK_HOLD          5

/*
 * Dead zone: |error| <= 15.
 */
#define BALL_DEADZONE_PX        30

/*
 * Minimum push step size (us).
 */
#define BALL_MIN_STEP_US        20U

/*
 * Print period (ms) and heartbeat period (ms).
 */
#define BALL_PRINT_PERIOD_MS    50U
#define BALL_HEARTBEAT_MS       2000U

/*
 * When ball is lost for this many consecutive frames, return servo
 * to center position.
 */
#define BALL_LOST_TIMEOUT_FRAMES 10U

static bool     s_initialized        = false;
static bool     s_vision_ok          = false;
static float    s_servo_pulse        = (float)SERVO_CENTER_US;
static uint32_t s_last_print_ms      = 0U;
static uint32_t s_last_heartbeat_ms  = 0U;
static uint32_t s_lost_frames        = 0U;
static int32_t  s_last_cx            = 0;
static int32_t  s_last_error         = 0;
static int32_t  s_prev_cx            = 0;
static int32_t  s_speed              = 0;
static bool     s_has_target         = false;
static bool     s_braking            = false;
static uint32_t s_stuck_counter       = 0U;
static uint32_t s_stuck_hold_frames    = 0U;
static uint16_t s_target_cx          = BALL_TARGET_CX_A;
static uint32_t s_cycle_start_ms     = 0U;

static uint16_t Servo_ClampPulse(int32_t raw, bool braking)
{
    int32_t min_us = braking ? (int32_t)SERVO_BRAKE_MIN_US
                             : (int32_t)SERVO_PUSH_MIN_US;
    int32_t max_us = braking ? (int32_t)SERVO_BRAKE_MAX_US
                             : (int32_t)SERVO_PUSH_MAX_US;
    if (raw < min_us) return (uint16_t)min_us;
    if (raw > max_us) return (uint16_t)max_us;
    return (uint16_t)raw;
}

bool Test_BallBalance_Init(void)
{
    s_initialized   = false;
    s_vision_ok     = false;
    s_servo_pulse   = (float)SERVO_CENTER_US;
    s_last_print_ms     = 0U;
    s_last_heartbeat_ms = 0U;
    s_lost_frames       = 0U;
    s_prev_cx           = 0;
    s_speed             = 0;
    s_braking           = false;
    s_stuck_counter     = 0U;
    s_stuck_hold_frames = 0U;
    s_target_cx         = BALL_TARGET_CX_A;
    s_cycle_start_ms    = 0U;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,START,MODE=CYCLE_%u_%u_HOLD_%lums,"
        "DEADZONE=%d,"
        "PUSH_KP=%.2f,BRAKE_GAIN=%.1f,"
        "BRAKE_BASE=%u,SPEED_FACT=%.1f,"
        "LIMS=PUSH_%u_%u_BRAKE_%u_%u\r\n",
        (unsigned int)BALL_TARGET_CX_A,
        (unsigned int)BALL_TARGET_CX_B,
        (unsigned long)BALL_CYCLE_HOLD_MS,
        BALL_DEADZONE_PX,
        (double)BALL_PUSH_KP,
        (double)BALL_BRAKE_GAIN,
        (unsigned int)BALL_BRAKE_BASE_US,
        (double)BALL_BRAKE_SPEED_FACTOR,
        (unsigned int)SERVO_PUSH_MIN_US,
        (unsigned int)SERVO_PUSH_MAX_US,
        (unsigned int)SERVO_BRAKE_MIN_US,
        (unsigned int)SERVO_BRAKE_MAX_US);

    /* Init servo at center position before enabling */
    if (!BSP_Servo_Init())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_INIT\r\n");
        return false;
    }

    if (!BSP_Servo_SetPulseUs(SERVO_CENTER_US))
    {
        (void)BSP_Debug_Printf("ERR,SERVO_SET_CENTER\r\n");
        return false;
    }

    if (!BSP_Servo_Enable())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_ENABLE\r\n");
        return false;
    }

    (void)BSP_Debug_Printf(
        "OK,SERVO,PULSE_US=%u,EN=%u\r\n",
        (unsigned int)BSP_Servo_GetPulseUs(),
        BSP_Servo_IsEnabled() ? 1U : 0U);

    /* Init vision UART */
    if (!BSP_VisionUart_Init())
    {
        (void)BSP_Debug_Printf("ERR,VISION_UART_INIT\r\n");
        /* Non-fatal: keep running without vision. */
    }
    else
    {
        s_vision_ok = true;
        (void)BSP_Debug_Printf("OK,VISION_UART_INIT\r\n");
    }

    s_last_print_ms     = HAL_GetTick();
    s_last_heartbeat_ms = s_last_print_ms;
    s_initialized       = true;
    return true;
}

void Test_BallBalance_Update(void)
{
    uint32_t now_ms;
    int32_t raw_pulse;
    uint16_t clamped_pulse;

    BSP_DebugUart_Process();
    BSP_VisionUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    /* ---- read vision: update persistent state ---- */
    if (s_vision_ok && BSP_VisionUart_HasNewDetection())
    {
        BspVisionDetection_t det = BSP_VisionUart_GetDetection();

        if (det.has_target)
        {
            s_last_cx    = (int32_t)det.cx;
            /* cx < target → ball left → error>0 → pulse increase → ball right */
            s_last_error = (int32_t)s_target_cx - s_last_cx;
            s_has_target = true;
            s_lost_frames = 0U;

            /* Speed: positive = moving right, negative = moving left */
            if (s_prev_cx != 0)
            {
                s_speed = s_last_cx - s_prev_cx;
            }
            s_prev_cx = s_last_cx;

            if (abs(s_last_error) <= BALL_DEADZONE_PX)
            {
                /* In dead zone: hold level */
                s_braking = false;
                if ((uint16_t)s_servo_pulse != SERVO_CENTER_US)
                {
                    (void)BSP_Servo_SetPulseUs(SERVO_CENTER_US);
                    s_servo_pulse = (float)SERVO_CENTER_US;
                }
            }
            else
            {
                int32_t delta;
                bool moving_to_target;

                /*
                 * Is ball moving toward target?
                 * error=target-cx: error>0=ball left, error<0=ball right
                 * Ball left (error>0) + moving right (speed>0) → toward
                 * Ball right (error<0) + moving left (speed<0) → toward
                 */
                moving_to_target =
                    (s_last_error > 0 && s_speed > 0) ||
                    (s_last_error < 0 && s_speed < 0);

                if (moving_to_target)
                {
                    /* Brake distance = how far ahead to start braking */
                    int32_t brake_dist =
                        (int32_t)((float)abs(s_speed) * BALL_BRAKE_GAIN);

                    if ((int32_t)abs(s_last_error) < brake_dist)
                    {
                        /* BRAKING: reverse tilt to slow ball down */
                        int32_t brake = (int32_t)BALL_BRAKE_BASE_US +
                            (int32_t)((float)abs(s_speed) * BALL_BRAKE_SPEED_FACTOR);

                        /*
                         * Ball moving right (speed>0): brake by tilting LEFT
                         * Ball moving left (speed<0): brake by tilting RIGHT
                         */
                        if (s_speed > 0)
                        {
                            delta = -(int32_t)brake;
                        }
                        else
                        {
                            delta = (int32_t)brake;
                        }

                        s_braking = true;
                    }
                    else
                    {
                        /* Moving toward target but far away: keep pushing */
                        delta = (int32_t)(BALL_PUSH_KP * (float)s_last_error);
                        s_braking = false;
                    }
                }
                else
                {
                    /* Ball stopped or moving away: push toward target */
                    delta = (int32_t)(BALL_PUSH_KP * (float)s_last_error);
                    s_braking = false;
                }

                /* Stuck detection: wait N frames, then boost for HOLD frames */
                if (s_stuck_hold_frames > 0U)
                {
                    /* Currently in boost hold phase */
                    s_stuck_hold_frames--;
                    if (delta > 0)
                        delta += (int32_t)BALL_STUCK_BOOST_US;
                    else
                        delta -= (int32_t)BALL_STUCK_BOOST_US;
                }
                else if (abs(s_speed) < BALL_STUCK_SPEED_PX && !s_braking)
                {
                    s_stuck_counter++;
                    if (s_stuck_counter >= BALL_STUCK_WAIT)
                    {
                        s_stuck_counter      = 0U;
                        s_stuck_hold_frames  = BALL_STUCK_HOLD;
                        if (delta > 0)
                            delta += (int32_t)BALL_STUCK_BOOST_US;
                        else
                            delta -= (int32_t)BALL_STUCK_BOOST_US;
                    }
                }
                else
                {
                    s_stuck_counter      = 0U;
                    s_stuck_hold_frames  = 0U;
                }

                /* Minimum step */
                if ((delta > 0) && (delta < (int32_t)BALL_MIN_STEP_US))
                {
                    delta = (int32_t)BALL_MIN_STEP_US;
                }
                else if ((delta < 0) && (delta > -(int32_t)BALL_MIN_STEP_US))
                {
                    delta = -(int32_t)BALL_MIN_STEP_US;
                }

                raw_pulse      = (int32_t)SERVO_CENTER_US + delta;
                clamped_pulse  = Servo_ClampPulse(raw_pulse, s_braking);

                if (clamped_pulse != (uint16_t)s_servo_pulse)
                {
                    if (BSP_Servo_SetPulseUs(clamped_pulse))
                    {
                        s_servo_pulse = (float)clamped_pulse;
                    }
                }
            }
        }
        else
        {
            s_has_target = false;
            s_lost_frames++;
        }
    }

    /* ---- cycle: hold at target for HOLD_MS before switching ---- */
    if (abs(s_last_error) <= BALL_DEADZONE_PX)
    {
        /* Ball in dead zone: start or continue hold timer */
        if (s_cycle_start_ms == 0U)
        {
            s_cycle_start_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s_cycle_start_ms) >= BALL_CYCLE_HOLD_MS)
        {
            /* Hold time elapsed, switch target */
            s_cycle_start_ms = 0U;
            s_stuck_counter     = 0U;
            s_stuck_hold_frames = 0U;

            if (s_target_cx == BALL_TARGET_CX_A)
            {
                s_target_cx = BALL_TARGET_CX_B;
            }
            else
            {
                s_target_cx = BALL_TARGET_CX_A;
            }

            (void)BSP_Debug_Printf(
                "BALL_BAL,CYCLE,NEW_TARGET=%u\r\n",
                (unsigned int)s_target_cx);
        }
    }
    else
    {
        /* Ball left dead zone: reset hold timer */
        s_cycle_start_ms = 0U;
    }

    /* ---- lost timeout: return to center ---- */
    if (s_lost_frames >= BALL_LOST_TIMEOUT_FRAMES)
    {
        if ((uint16_t)s_servo_pulse != SERVO_CENTER_US)
        {
            (void)BSP_Servo_SetPulseUs(SERVO_CENTER_US);
            s_servo_pulse = (float)SERVO_CENTER_US;
            (void)BSP_Debug_Printf(
                "BALL_BAL,LOST_RETURN_CENTER,PULSE=%u\r\n",
                (unsigned int)SERVO_CENTER_US);
        }
        s_lost_frames = 0U;
    }

    /* ---- heartbeat ---- */
    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >= BALL_HEARTBEAT_MS)
    {
        s_last_heartbeat_ms = now_ms;
        (void)BSP_Debug_Printf(
            "BALL_BAL,HB,PULSE=%u,FRAMES=%lu,ERR=%lu,VISION=%u\r\n",
            (unsigned int)s_servo_pulse,
            (unsigned long)BSP_VisionUart_GetFrameCount(),
            (unsigned long)BSP_VisionUart_GetErrorCount(),
            s_vision_ok ? 1U : 0U);
    }

    /* ---- periodic print ---- */
    if ((uint32_t)(now_ms - s_last_print_ms) >= BALL_PRINT_PERIOD_MS)
    {
        s_last_print_ms = now_ms;

        if (s_has_target)
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,CX=%ld,ERR=%+ld,SPD=%+ld,%s,PULSE=%u\r\n",
                (long)s_last_cx,
                (long)s_last_error,
                (long)s_speed,
                s_braking ? "BRAKE" : "PUSH",
                (unsigned int)s_servo_pulse);
        }
        else
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,NO_TARGET,LOST=%lu,PULSE=%u\r\n",
                (unsigned long)s_lost_frames,
                (unsigned int)s_servo_pulse);
        }
    }
}

void Test_BallBalance_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    BSP_Servo_Disable();
    s_initialized = false;

    (void)BSP_Debug_Printf(
        "BALL_BAL,STOP,PULSE=%u\r\n",
        (unsigned int)s_servo_pulse);
}

bool Test_BallBalance_IsInitialized(void)
{
    return s_initialized;
}
