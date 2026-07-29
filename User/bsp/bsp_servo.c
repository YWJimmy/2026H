#include "bsp_servo.h"

#include "tim.h"

/*
 * 舵机方向修正：
 * 0：逻辑脉宽直接输出。
 * 1：以 1500 us 为中心进行镜像。
 *
 * 例如开启反向后：
 * 1450 us -> 1550 us
 * 1550 us -> 1450 us
 */
#define BSP_SERVO_REVERSED           0U

/* 当前 CubeMX 配置：TIM1 时钟 168 MHz，PSC=167，ARR=19999。 */
#define BSP_SERVO_TIM_PRESCALER      ((uint32_t)167U)
#define BSP_SERVO_TIM_PERIOD         ((uint32_t)19999U)

static bool s_initialized = false;
static bool s_enabled = false;
static uint16_t s_logical_pulse_us = BSP_SERVO_PULSE_CENTER_US;

static uint16_t Servo_ClampPulseUs(int32_t pulse_us)
{
    if (pulse_us < (int32_t)BSP_SERVO_PULSE_MIN_US)
    {
        return BSP_SERVO_PULSE_MIN_US;
    }

    if (pulse_us > (int32_t)BSP_SERVO_PULSE_MAX_US)
    {
        return BSP_SERVO_PULSE_MAX_US;
    }

    return (uint16_t)pulse_us;
}

static uint16_t Servo_LogicalToHardwarePulseUs(uint16_t logical_pulse_us)
{
    int32_t hardware_pulse_us;

    if (BSP_SERVO_REVERSED == 0U)
    {
        return logical_pulse_us;
    }

    hardware_pulse_us =
        (int32_t)BSP_SERVO_PULSE_CENTER_US -
        ((int32_t)logical_pulse_us -
         (int32_t)BSP_SERVO_PULSE_CENTER_US);

    return Servo_ClampPulseUs(hardware_pulse_us);
}

static void Servo_WriteCompare(uint16_t logical_pulse_us)
{
    uint16_t hardware_pulse_us =
        Servo_LogicalToHardwarePulseUs(logical_pulse_us);

    /*
     * TIM1 已配置为 1 us/计数，因此 CCR 数值可直接使用微秒数。
     */
    __HAL_TIM_SET_COMPARE(&htim1,
                          TIM_CHANNEL_1,
                          hardware_pulse_us);
}

bool BSP_Servo_Init(void)
{
    s_initialized = false;
    s_enabled = false;
    s_logical_pulse_us = BSP_SERVO_PULSE_CENTER_US;

    /*
     * 防止 CubeMX 定时器配置被误改后，BSP 仍以“1 us/计数”运行。
     */
    if ((htim1.Instance != TIM1) ||
        (htim1.Init.Prescaler != BSP_SERVO_TIM_PRESCALER) ||
        (htim1.Init.Period != BSP_SERVO_TIM_PERIOD))
    {
        return false;
    }

    /*
     * 初始化不启动 PWM。忽略 Stop 返回值，因为定时器可能尚未启动。
     */
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    Servo_WriteCompare(BSP_SERVO_PULSE_CENTER_US);

    s_initialized = true;
    return true;
}

bool BSP_Servo_Enable(void)
{
    if (!s_initialized)
    {
        return false;
    }

    if (s_enabled)
    {
        return true;
    }

    Servo_WriteCompare(s_logical_pulse_us);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    s_enabled = true;
    return true;
}

void BSP_Servo_Disable(void)
{
    if (!s_initialized)
    {
        return;
    }

    s_logical_pulse_us = BSP_SERVO_PULSE_CENTER_US;
    Servo_WriteCompare(s_logical_pulse_us);

    if (s_enabled)
    {
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    }

    s_enabled = false;
}

bool BSP_Servo_IsInitialized(void)
{
    return s_initialized;
}

bool BSP_Servo_IsEnabled(void)
{
    return s_initialized && s_enabled;
}

bool BSP_Servo_SetPulseUs(uint16_t pulse_us)
{
    if (!s_initialized)
    {
        return false;
    }

    s_logical_pulse_us = Servo_ClampPulseUs((int32_t)pulse_us);

    if (s_enabled)
    {
        Servo_WriteCompare(s_logical_pulse_us);
    }

    return true;
}

bool BSP_Servo_SetOffsetUs(int16_t offset_us)
{
    int32_t pulse_us;

    if (!s_initialized)
    {
        return false;
    }

    pulse_us =
        (int32_t)BSP_SERVO_PULSE_CENTER_US +
        (int32_t)offset_us;

    s_logical_pulse_us = Servo_ClampPulseUs((int32_t)pulse_us);

    if (s_enabled)
    {
        Servo_WriteCompare(s_logical_pulse_us);
    }

    return true;
}

uint16_t BSP_Servo_GetPulseUs(void)
{
    return s_logical_pulse_us;
}
