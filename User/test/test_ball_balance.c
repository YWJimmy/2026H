#include "test_ball_balance.h"

#include "bsp_debug_uart.h"
#include "bsp_servo.h"
#include "bsp_vision_uart.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdlib.h>

/*
 * Target ball center X in 1280x960 coordinates.
 */
#define BALL_TARGET_CX          500

/*
 * Servo pulse width limits (us), measured by user.
 * 1600 = stable/level, 1400 = ball left, 1800 = ball right.
 */
#define SERVO_CENTER_US         1600U
#define SERVO_MIN_US            1400U
#define SERVO_MAX_US            1800U

/*
 * PI-controller gains.
 *   delta_us = Kp * error + Ki * integral
 */
#define BALL_KP                 1.0f
#define BALL_KI                 0.005f

/*
 * Integral anti-windup: clamp integral to +/- this value.
 */
#define BALL_INTEGRAL_MAX       4000.0f

/*
 * Dead zone: |error| <= 30 → ball within 470..530, hold position.
 */
#define BALL_DEADZONE_PX        30

/*
 * Minimum step size (us) on total delta.
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
static int32_t  s_prev_error         = 0;
static float    s_integral           = 0.0f;
static bool     s_has_target         = false;

static uint16_t Servo_ClampPulse(int32_t raw)
{
    if (raw < (int32_t)SERVO_MIN_US) return SERVO_MIN_US;
    if (raw > (int32_t)SERVO_MAX_US) return SERVO_MAX_US;
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
    s_integral          = 0.0f;
    s_prev_error        = 0;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,START,TARGET_CX=%d,"
        "SERVO_CENTER=%u,SERVO_MIN=%u,SERVO_MAX=%u,"
        "KP=%.2f,KI=%.3f,DEADZONE=%d\r\n",
        BALL_TARGET_CX,
        (unsigned int)SERVO_CENTER_US,
        (unsigned int)SERVO_MIN_US,
        (unsigned int)SERVO_MAX_US,
        (double)BALL_KP,
        (double)BALL_KI,
        BALL_DEADZONE_PX);

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
            s_last_error = BALL_TARGET_CX - s_last_cx;
            s_has_target = true;
            s_lost_frames = 0U;

            if (abs(s_last_error) > BALL_DEADZONE_PX)
            {
                int32_t delta;

                /* Reset integral on error sign change (crossed target) */
                if ((s_last_error > 0 && s_prev_error < 0) ||
                    (s_last_error < 0 && s_prev_error > 0))
                {
                    s_integral = 0.0f;
                }

                /* Accumulate integral with anti-windup */
                s_integral += (float)s_last_error;
                if (s_integral > BALL_INTEGRAL_MAX)
                {
                    s_integral = BALL_INTEGRAL_MAX;
                }
                else if (s_integral < -BALL_INTEGRAL_MAX)
                {
                    s_integral = -BALL_INTEGRAL_MAX;
                }

                delta = (int32_t)(
                    BALL_KP * (float)s_last_error +
                    BALL_KI * s_integral);

                /* Minimum step */
                if ((delta > 0) && (delta < (int32_t)BALL_MIN_STEP_US))
                {
                    delta = (int32_t)BALL_MIN_STEP_US;
                }
                else if ((delta < 0) && (delta > -(int32_t)BALL_MIN_STEP_US))
                {
                    delta = -(int32_t)BALL_MIN_STEP_US;
                }

                raw_pulse = (int32_t)SERVO_CENTER_US + delta;
                clamped_pulse = Servo_ClampPulse(raw_pulse);

                if (clamped_pulse != (uint16_t)s_servo_pulse)
                {
                    if (BSP_Servo_SetPulseUs(clamped_pulse))
                    {
                        s_servo_pulse = (float)clamped_pulse;
                    }
                }

                s_prev_error = s_last_error;
            }
            else
            {
                /* In dead zone: stop accumulating, hold position */
                s_integral   = 0.0f;
                s_prev_error = 0;
            }
        }
        else
        {
            s_has_target = false;
            s_lost_frames++;
        }
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
                "BALL_BAL,CX=%ld,ERROR=%ld,INT=%ld,PULSE=%u\r\n",
                (long)s_last_cx,
                (long)s_last_error,
                (long)s_integral,
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
