#include "test_motor_open_loop.h"

#include "bsp_debug_uart.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define TEST_MOTOR_STEP_HOLD_MS       2000U
#define TEST_MOTOR_PRINT_PERIOD_MS    100U

static const int16_t s_pwm_steps[] =
{
    0,
    500,
    1000,
    1500,
    2000,
    2500,
    3000,
    3500,
    4000,
    4500,
    5000,
    0
};

#define TEST_MOTOR_STEP_COUNT \
    ((uint8_t)(sizeof(s_pwm_steps) / sizeof(s_pwm_steps[0])))

static bool s_initialized = false;
static bool s_finished = false;
static uint8_t s_step_index = 0U;
static uint32_t s_step_start_ms = 0U;
static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_sample_ms = 0U;

static void Test_MotorOpenLoop_ApplyStep(void)
{
    int16_t pwm = s_pwm_steps[s_step_index];

    (void)BSP_Motor_Set(pwm, pwm);

    (void)BSP_Debug_Printf(
        "MOPEN,STEP=%u,PWM=%d\r\n",
        (unsigned int)s_step_index,
        (int)pwm);
}

bool Test_MotorOpenLoop_Init(void)
{
    s_initialized = false;
    s_finished = false;
    s_step_index = 0U;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!BSP_Motor_Init())
    {
        (void)BSP_Debug_Printf("ERR,MOTOR_INIT\r\n");
        return false;
    }

    if (!BSP_Encoder_Init())
    {
        (void)BSP_Debug_Printf("ERR,ENCODER_INIT\r\n");
        return false;
    }

    if (!BSP_Motor_Enable(true))
    {
        (void)BSP_Debug_Printf("ERR,MOTOR_ENABLE\r\n");
        return false;
    }

    BSP_Encoder_Reset();

    s_step_start_ms = HAL_GetTick();
    s_last_print_ms = s_step_start_ms;
    s_last_sample_ms = s_step_start_ms;
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,MOTOR_OPEN_LOOP,START,HOLD_MS=%lu,PRINT_MS=%lu,"
        "NOTICE=LIFT_WHEELS\r\n",
        (unsigned long)TEST_MOTOR_STEP_HOLD_MS,
        (unsigned long)TEST_MOTOR_PRINT_PERIOD_MS);

    Test_MotorOpenLoop_ApplyStep();
    return true;
}

void Test_MotorOpenLoop_Update(void)
{
    uint32_t now_ms;
    uint32_t sample_dt_ms;
    BspEncoderSample_t sample;
    int32_t left_cps;
    int32_t right_cps;

    BSP_DebugUart_Process();

    if ((!s_initialized) || s_finished)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - s_last_print_ms) >=
        TEST_MOTOR_PRINT_PERIOD_MS)
    {
        sample_dt_ms = (uint32_t)(now_ms - s_last_sample_ms);
        s_last_print_ms = now_ms;
        s_last_sample_ms = now_ms;

        if (BSP_Encoder_Sample(&sample) && (sample_dt_ms > 0U))
        {
            left_cps =
                (int32_t)(((int64_t)sample.left_delta * 1000LL) /
                          (int64_t)sample_dt_ms);

            right_cps =
                (int32_t)(((int64_t)sample.right_delta * 1000LL) /
                          (int64_t)sample_dt_ms);

            (void)BSP_Debug_Printf(
                "MOPEN,PWM=%d,LD=%d,RD=%d,LCPS=%ld,RCPS=%ld,"
                "LT=%ld,RT=%ld\r\n",
                (int)s_pwm_steps[s_step_index],
                (int)sample.left_delta,
                (int)sample.right_delta,
                (long)left_cps,
                (long)right_cps,
                (long)sample.left_total,
                (long)sample.right_total);
        }
    }

    if ((uint32_t)(now_ms - s_step_start_ms) <
        TEST_MOTOR_STEP_HOLD_MS)
    {
        return;
    }

    s_step_start_ms = now_ms;
    s_step_index++;

    if (s_step_index >= TEST_MOTOR_STEP_COUNT)
    {
        Test_MotorOpenLoop_Stop();
        s_finished = true;
        return;
    }

    Test_MotorOpenLoop_ApplyStep();
}

void Test_MotorOpenLoop_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    BSP_Motor_BrakeAll();
    (void)BSP_Motor_Enable(false);

    s_initialized = false;

    (void)BSP_Debug_Printf(
        "TEST,MOTOR_OPEN_LOOP,STOP\r\n");
}

bool Test_MotorOpenLoop_IsInitialized(void)
{
    return s_initialized;
}
