#include "test_task4_ab_hold.h"

#include "bsp_debug_uart.h"
#include "bsp_servo.h"
#include "bsp_vision_uart.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

/* Task parameters */
#define TASK4_DISTANCE_MM       1500
#define TASK4_TIME_LIMIT_MS     8000U
#define TASK4_SPEED_MM_S        300
#define TASK4_BALL_CENTER_CX    653
#define TASK4_SERVO_CENTER_US   1650U
#define TASK4_SERVO_MIN_US      1300U
#define TASK4_SERVO_MAX_US      2000U
#define TASK4_KP                0.8f
#define TASK4_DEADZONE_PX       30
#define TASK4_BRAKE_GAIN        5.0f
#define TASK4_BRAKE_BASE_US     80U
#define TASK4_BRAKE_SPEED_US    7.0f
#define TASK4_MIN_STEP_US       15U
#define TASK4_FWD_BIAS_US       60U
#define TASK4_BRAKE_BIAS_US      50U
#define TASK4_PRINT_PERIOD_MS   200U
#define TASK4_HEARTBEAT_MS      1000U
#define TASK4_BALL_LOST_LIMIT   10U

/* Encoder: 1468 counts/rev, wheel 204204 um/rev */
#define TASK4_ENC_UM_PER_CNT     139
#define TASK4_UM_TO_MM(um)       ((um) / 1000)

typedef enum {
    TASK4_STATE_ARMED = 0,
    TASK4_STATE_DRIVING,
    TASK4_STATE_DONE
} Task4State_t;

static bool s_initialized = false;
static Task4State_t s_state = TASK4_STATE_ARMED;
static uint32_t s_state_start_ms = 0U;
static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_hb_ms = 0U;
static bool s_passed = false;
static uint32_t s_elapsed_ms = 0U;
static int32_t s_last_cx = 0;
static int32_t s_last_error = 0;
static int32_t s_max_error = 0;
static int32_t s_start_left = 0;
static int32_t s_start_right = 0;
static bool s_enc_zeroed = false;
static uint16_t s_servo_pulse = TASK4_SERVO_CENTER_US;
static bool s_vision_ok = false;
static uint32_t s_lost_frames = 0U;
static int32_t s_prev_cx = 0;
static int32_t s_speed = 0;

static uint16_t Servo_Clamp(int32_t raw) {
    if (raw < (int32_t)TASK4_SERVO_MIN_US) return TASK4_SERVO_MIN_US;
    if (raw > (int32_t)TASK4_SERVO_MAX_US) return TASK4_SERVO_MAX_US;
    return (uint16_t)raw;
}

bool Test_Task4ABHold_Init(void)
{
    ChassisStatus_t cs;
    s_initialized = false;
    s_state = TASK4_STATE_ARMED;
    s_passed = false;
    s_max_error = 0;
    s_enc_zeroed = false;
    s_servo_pulse = TASK4_SERVO_CENTER_US;
    s_vision_ok = false;
    s_lost_frames = 0U;

    if (!BSP_DebugUart_Init()) return false;
    (void)BSP_Debug_Printf("TASK4,INIT,DIST=%dmm,TIME=%lu,SPEED=%d\r\n",
        TASK4_DISTANCE_MM, (unsigned long)TASK4_TIME_LIMIT_MS, TASK4_SPEED_MM_S);

    /* Servo */
    if (!BSP_Servo_Init()) {
        (void)BSP_Debug_Printf("ERR,SERVO_INIT\r\n"); return false;
    }
    BSP_Servo_SetPulseUs(TASK4_SERVO_CENTER_US);
    BSP_Servo_Enable();

    /* Vision */
    if (BSP_VisionUart_Init()) {
        s_vision_ok = true;
        (void)BSP_Debug_Printf("OK,VISION\r\n");
    } else {
        (void)BSP_Debug_Printf("WARN,VISION_INIT_FAIL\r\n");
    }

    /* Chassis */
    if (!Chassis_Init()) {
        (void)BSP_Debug_Printf("ERR,CHASSIS\r\n"); return false;
    }
    if (!Chassis_Enable(true)) {
        (void)BSP_Debug_Printf("ERR,CHASSIS_EN\r\n"); return false;
    }
    if (Chassis_GetStatus(&cs)) {
        s_start_left  = cs.left_total;
        s_start_right = cs.right_total;
        s_enc_zeroed = true;
    }

    (void)BSP_Debug_Printf("TASK4,OK,ARMED,SERVO=%u,VISION=%u\r\n",
        (unsigned int)BSP_Servo_GetPulseUs(), s_vision_ok ? 1U : 0U);

    s_state_start_ms = HAL_GetTick();
    s_last_print_ms = s_state_start_ms;
    s_last_hb_ms = s_state_start_ms;
    s_initialized = true;
    return true;
}

void Test_Task4ABHold_Update(void)
{
    BspVisionDetection_t det;
    ChassisStatus_t cs;
    uint32_t now_ms = HAL_GetTick();
    int32_t dist_mm = 0;

    BSP_DebugUart_Process();
    BSP_VisionUart_Process();

    if (!s_initialized) return;

    /* ---- Ball balance (P + brake) ---- */
    if (s_vision_ok && BSP_VisionUart_HasNewDetection()) {
        det = BSP_VisionUart_GetDetection();
        if (det.has_target) {
            s_last_cx = det.cx;
            s_last_error = (int32_t)TASK4_BALL_CENTER_CX - (int32_t)det.cx;
            s_lost_frames = 0U;
            s_speed = (s_prev_cx != 0) ? ((int32_t)det.cx - s_prev_cx) : 0;
            s_prev_cx = (int32_t)det.cx;

            int32_t aerr = (s_last_error < 0) ? -s_last_error : s_last_error;
            if (aerr > s_max_error) s_max_error = aerr;

            /* Driving bias timing:
             *   0.0~1.5s: left tilt 200us
             *   1.5~2.0s: right tilt 200us
             *   >2.0s:   normal 60us left
             *   near end: strong right tilt 200us when within 1s of arrival
             */
            int32_t fwd_bias = 0;
            if (s_state == TASK4_STATE_DRIVING) {
                if (s_elapsed_ms < 1500U) {
                    fwd_bias = 200;   /* left tilt */
                } else if (s_elapsed_ms < 1700U) {
                    fwd_bias = -200;  /* right tilt */
                } else {
                    fwd_bias = 0;  /* no tilt after 2s */
                }
            } else if (s_state == TASK4_STATE_DONE) {
                /* After stop: keep right tilt for 1.5s */
                uint32_t done_ms = (uint32_t)(now_ms - s_state_start_ms);
                if (done_ms < 1500U) {
                    fwd_bias = -200;
                }
            }
            int32_t active_center = (int32_t)TASK4_SERVO_CENTER_US - fwd_bias;

            if (aerr <= TASK4_DEADZONE_PX) {
                if (s_servo_pulse != (uint16_t)active_center) {
                    s_servo_pulse = (uint16_t)active_center;
                    BSP_Servo_SetPulseUs(s_servo_pulse);
                }
            } else {
                int32_t delta;
                bool toward = (s_last_error > 0 && s_speed > 0) ||
                              (s_last_error < 0 && s_speed < 0);

                if (toward) {
                    /* Full brake when moving toward target */
                    int32_t brake_dist = (int32_t)((float)(s_speed < 0 ? -s_speed : s_speed) * TASK4_BRAKE_GAIN);
                    int32_t brake = (int32_t)TASK4_BRAKE_BASE_US +
                        (int32_t)((float)(s_speed < 0 ? -s_speed : s_speed) * TASK4_BRAKE_SPEED_US);
                    int32_t brake_bias = 0;
                    /* Strong right tilt when braking near destination */
                    if (s_state == TASK4_STATE_DRIVING) {
                        if (dist_mm > TASK4_DISTANCE_MM - 300) {
                            brake_bias = 200;  /* near end: full right tilt */
                        } else {
                            brake_bias = (int32_t)TASK4_BRAKE_BIAS_US;
                        }
                    }
                    brake += brake_bias;
                    if (aerr < brake_dist) {
                        delta = (s_speed > 0) ? -brake : brake;
                    } else {
                        delta = (int32_t)(TASK4_KP * (float)s_last_error);
                    }
                } else if (s_speed != 0) {
                    /* Moving away: half brake to dampen oscillation */
                    int32_t damp = (int32_t)TASK4_BRAKE_BASE_US / 2 +
                        (int32_t)((float)(s_speed < 0 ? -s_speed : s_speed) * TASK4_BRAKE_SPEED_US / 2.0f);
                    delta = (s_speed > 0) ? -damp : damp;
                } else {
                    delta = (int32_t)(TASK4_KP * (float)s_last_error);
                }

                if (delta > 0 && delta < (int32_t)TASK4_MIN_STEP_US) delta = TASK4_MIN_STEP_US;
                if (delta < 0 && delta > -(int32_t)TASK4_MIN_STEP_US) delta = -(int32_t)TASK4_MIN_STEP_US;

                s_servo_pulse = Servo_Clamp(active_center + delta);
                BSP_Servo_SetPulseUs(s_servo_pulse);
            }
        } else {
            s_lost_frames++;
        }
    }

    /* ---- Chassis ---- */
    Chassis_Update();
    if (Chassis_GetStatus(&cs)) {
        int32_t dl = cs.left_total  - s_start_left;
        int32_t dr = cs.right_total - s_start_right;
        int32_t avg = (dl < 0 ? -dl : dl) + (dr < 0 ? -dr : dr);
        avg /= 2;
        dist_mm = TASK4_UM_TO_MM(avg * TASK4_ENC_UM_PER_CNT);
    }

    /* ---- State machine ---- */
    switch (s_state) {
    case TASK4_STATE_ARMED:
        if (!Chassis_SetVelocity(TASK4_SPEED_MM_S, 0)) {
            (void)BSP_Debug_Printf("ERR,SET_VEL\r\n");
            s_passed = false; s_state_start_ms = now_ms; s_state = TASK4_STATE_DONE; break;
        }
        (void)BSP_Debug_Printf("TASK4,GO\r\n");
        s_state = TASK4_STATE_DRIVING;
        s_state_start_ms = now_ms;
        break;

    case TASK4_STATE_DRIVING:
        s_elapsed_ms = (uint32_t)(now_ms - s_state_start_ms);
        if (dist_mm >= TASK4_DISTANCE_MM) {
            Chassis_Stop();
            s_passed = (s_elapsed_ms <= TASK4_TIME_LIMIT_MS) && (s_max_error <= 100);
            (void)BSP_Debug_Printf("TASK4,FINISH,DIST=%ld,T=%lu,PASS=%u,MAX_ERR=%ld\r\n",
                (long)dist_mm, (unsigned long)s_elapsed_ms, s_passed ? 1U : 0U, (long)s_max_error);
            s_state_start_ms = now_ms; s_state = TASK4_STATE_DONE;
        } else if (s_elapsed_ms >= TASK4_TIME_LIMIT_MS) {
            Chassis_Stop();
            (void)BSP_Debug_Printf("TASK4,FAIL,TIMEOUT,DIST=%ld\r\n", (long)dist_mm);
            s_passed = false; s_state_start_ms = now_ms; s_state = TASK4_STATE_DONE;
        } else if (s_lost_frames > TASK4_BALL_LOST_LIMIT) {
            Chassis_Stop();
            (void)BSP_Debug_Printf("TASK4,FAIL,BALL_LOST\r\n");
            s_passed = false; s_state_start_ms = now_ms; s_state = TASK4_STATE_DONE;
        }
        break;

    case TASK4_STATE_DONE:
        break;
    }

    /* Heartbeat */
    if ((uint32_t)(now_ms - s_last_hb_ms) >= TASK4_HEARTBEAT_MS) {
        s_last_hb_ms = now_ms;
        (void)BSP_Debug_Printf("TASK4,HB,DIST=%ld,T=%lu,CX=%ld,ERR=%ld,PASS=%u\r\n",
            (long)dist_mm, (unsigned long)s_elapsed_ms, (long)s_last_cx, (long)s_last_error, s_passed ? 1U : 0U);
    }
    if ((uint32_t)(now_ms - s_last_print_ms) >= TASK4_PRINT_PERIOD_MS) {
        s_last_print_ms = now_ms;
        (void)BSP_Debug_Printf("TASK4,DIST=%ld,CX=%ld,ERR=%ld,T=%lu\r\n",
            (long)dist_mm, (long)s_last_cx, (long)s_last_error, (unsigned long)s_elapsed_ms);
    }
}

void Test_Task4ABHold_Stop(void) {
    if (!s_initialized) return;
    Chassis_Stop();
    BSP_Servo_Disable();
    s_initialized = false;
    (void)BSP_Debug_Printf("TASK4,STOP\r\n");
}

bool Test_Task4ABHold_IsInitialized(void)  { return s_initialized; }
bool Test_Task4ABHold_IsFinished(void)     { return s_state == TASK4_STATE_DONE; }
bool Test_Task4ABHold_IsPassed(void)       { return s_passed; }
uint32_t Test_Task4ABHold_GetElapsedMs(void) { return s_elapsed_ms; }
bool Test_Task4ABHold_IsTimerRunning(void) { return s_state == TASK4_STATE_DRIVING; }
