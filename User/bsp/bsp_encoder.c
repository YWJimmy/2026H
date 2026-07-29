#include "bsp_encoder.h"

#include "tim.h"

#include <limits.h>

/*
 * 编码器方向修正开关：
 * 0：保持 TIM 硬件计数方向。
 * 1：将该侧计数方向取反。
 *
 * 正方向统一定义为“车轮驱动小车向前时计数增加”。
 * 架空车轮测试后，哪一侧方向相反，就只修改对应宏。
 */
#define BSP_ENCODER_LEFT_REVERSED     1U
#define BSP_ENCODER_RIGHT_REVERSED    0U

/* TIM3、TIM4 当前均配置为 16 位满周期计数。 */
#define BSP_ENCODER_TIMER_PERIOD      ((uint32_t)0xFFFFU)

static bool s_initialized = false;

static uint16_t s_left_last_counter = 0U;
static uint16_t s_right_last_counter = 0U;

static int32_t s_left_total = 0;
static int32_t s_right_total = 0;

static int16_t Encoder_ApplyDirection(int16_t delta, bool reversed)
{
    if (!reversed)
    {
        return delta;
    }

    /* -32768 无法在 int16_t 中直接取正，采用饱和值避免溢出。 */
    if (delta == INT16_MIN)
    {
        return INT16_MAX;
    }

    return (int16_t)(-delta);
}

static int32_t Encoder_AddSaturated(int32_t total, int16_t delta)
{
    int64_t result = (int64_t)total + (int64_t)delta;

    if (result > INT32_MAX)
    {
        return INT32_MAX;
    }

    if (result < INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)result;
}

static int16_t Encoder_ReadDelta(TIM_HandleTypeDef *timer,
                                 uint16_t *last_counter,
                                 bool reversed)
{
    uint16_t current_counter;
    int16_t delta;

    current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(timer);

    /*
     * 对无符号 16 位差值再解释为有符号数，可自动处理 0/65535 回绕。
     * 前提是两次读取之间的真实增量绝对值小于 32768。
     */
    delta = (int16_t)(uint16_t)(current_counter - *last_counter);
    *last_counter = current_counter;

    return Encoder_ApplyDirection(delta, reversed);
}

bool BSP_Encoder_Init(void)
{
    s_initialized = false;

    /* 防止 CubeMX 定时器配置被误改后仍静默运行。 */
    if ((htim3.Instance != TIM3) ||
        (htim4.Instance != TIM4) ||
        (htim3.Init.Prescaler != 0U) ||
        (htim4.Init.Prescaler != 0U) ||
        (htim3.Init.Period != BSP_ENCODER_TIMER_PERIOD) ||
        (htim4.Init.Period != BSP_ENCODER_TIMER_PERIOD))
    {
        return false;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);

    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
        return false;
    }

    /* 启动后再次清零，保证软件基准和硬件计数完全一致。 */
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);

    s_left_last_counter = 0U;
    s_right_last_counter = 0U;
    s_left_total = 0;
    s_right_total = 0;
    s_initialized = true;

    return true;
}

bool BSP_Encoder_IsInitialized(void)
{
    return s_initialized;
}

void BSP_Encoder_Reset(void)
{
    if (!s_initialized)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);

    s_left_last_counter = 0U;
    s_right_last_counter = 0U;
    s_left_total = 0;
    s_right_total = 0;
}

int16_t BSP_Encoder_GetLeftDelta(void)
{
    int16_t delta;

    if (!s_initialized)
    {
        return 0;
    }

    delta = Encoder_ReadDelta(&htim3,
                              &s_left_last_counter,
                              (BSP_ENCODER_LEFT_REVERSED != 0U));
    s_left_total = Encoder_AddSaturated(s_left_total, delta);

    return delta;
}

int16_t BSP_Encoder_GetRightDelta(void)
{
    int16_t delta;

    if (!s_initialized)
    {
        return 0;
    }

    delta = Encoder_ReadDelta(&htim4,
                              &s_right_last_counter,
                              (BSP_ENCODER_RIGHT_REVERSED != 0U));
    s_right_total = Encoder_AddSaturated(s_right_total, delta);

    return delta;
}

int32_t BSP_Encoder_GetLeftTotal(void)
{
    return s_left_total;
}

int32_t BSP_Encoder_GetRightTotal(void)
{
    return s_right_total;
}
