#include "bsp_motor.h"

#include "main.h"
#include "tim.h"

/*
 * 电机方向修正开关：
 * 0：正指令时 IN1=1、IN2=0。
 * 1：正指令时 IN1=0、IN2=1。
 *
 * 当前先按 A 方案实现，即左右两侧均为 0。
 * 架空车轮测试后，哪一侧方向相反，就只把对应宏改为 1。
 */
#define BSP_MOTOR_LEFT_REVERSED     0U
#define BSP_MOTOR_RIGHT_REVERSED    0U

typedef enum
{
    MOTOR_SIDE_LEFT = 0,
    MOTOR_SIDE_RIGHT
} MotorSide_t;

static bool s_initialized = false;
static bool s_enabled = false;

static uint16_t Motor_GetMagnitude(int16_t command)
{
    int32_t magnitude = (int32_t)command;

    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }

    if (magnitude > BSP_MOTOR_PWM_MAX)
    {
        magnitude = BSP_MOTOR_PWM_MAX;
    }

    return (uint16_t)magnitude;
}

static void Motor_SetCompare(MotorSide_t side, uint16_t compare)
{
    if (compare > (uint16_t)BSP_MOTOR_PWM_MAX)
    {
        compare = (uint16_t)BSP_MOTOR_PWM_MAX;
    }

    if (side == MOTOR_SIDE_LEFT)
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, compare);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, compare);
    }
}

static void Motor_SetDirectionPins(MotorSide_t side, bool forward)
{
    bool reversed = false;

    if (side == MOTOR_SIDE_LEFT)
    {
        reversed = (BSP_MOTOR_LEFT_REVERSED != 0U);
    }
    else
    {
        reversed = (BSP_MOTOR_RIGHT_REVERSED != 0U);
    }

    if (reversed)
    {
        forward = !forward;
    }

    if (side == MOTOR_SIDE_LEFT)
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port,
                          MOTOR_L_IN1_Pin,
                          forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port,
                          MOTOR_L_IN2_Pin,
                          forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_R_IN1_GPIO_Port,
                          MOTOR_R_IN1_Pin,
                          forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_GPIO_Port,
                          MOTOR_R_IN2_Pin,
                          forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

static void Motor_BrakeDirect(MotorSide_t side)
{
    /* PWM=0 时 TB6612FNG 已进入短路刹车；同时固定 IN1=IN2=1。 */
    Motor_SetCompare(side, 0U);

    if (side == MOTOR_SIDE_LEFT)
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port,
                          MOTOR_L_IN1_Pin,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port,
                          MOTOR_L_IN2_Pin,
                          GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_R_IN1_GPIO_Port,
                          MOTOR_R_IN1_Pin,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_GPIO_Port,
                          MOTOR_R_IN2_Pin,
                          GPIO_PIN_SET);
    }
}

static void Motor_ApplyCommand(MotorSide_t side, int16_t command)
{
    if (command == 0)
    {
        Motor_BrakeDirect(side);
        return;
    }

    /* 方向切换前先清零 PWM，避免带占空比直接改变方向。 */
    Motor_SetCompare(side, 0U);
    Motor_SetDirectionPins(side, command > 0);
    Motor_SetCompare(side, Motor_GetMagnitude(command));
}

bool BSP_Motor_Init(void)
{
    s_initialized = false;
    s_enabled = false;

    /* 初始化过程中禁止电机输出。 */
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port,
                      MOTOR_STBY_Pin,
                      GPIO_PIN_RESET);

    Motor_BrakeDirect(MOTOR_SIDE_LEFT);
    Motor_BrakeDirect(MOTOR_SIDE_RIGHT);

    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
        return false;
    }

    Motor_BrakeDirect(MOTOR_SIDE_LEFT);
    Motor_BrakeDirect(MOTOR_SIDE_RIGHT);

    s_initialized = true;
    return true;
}

bool BSP_Motor_Enable(bool enable)
{
    if (!s_initialized)
    {
        return false;
    }

    Motor_BrakeDirect(MOTOR_SIDE_LEFT);
    Motor_BrakeDirect(MOTOR_SIDE_RIGHT);

    if (enable)
    {
        HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port,
                          MOTOR_STBY_Pin,
                          GPIO_PIN_SET);
        s_enabled = true;
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port,
                          MOTOR_STBY_Pin,
                          GPIO_PIN_RESET);
        s_enabled = false;
    }

    return true;
}

bool BSP_Motor_IsEnabled(void)
{
    return s_initialized && s_enabled;
}

bool BSP_Motor_SetLeft(int16_t command)
{
    if (!BSP_Motor_IsEnabled())
    {
        return false;
    }

    Motor_ApplyCommand(MOTOR_SIDE_LEFT, command);
    return true;
}

bool BSP_Motor_SetRight(int16_t command)
{
    if (!BSP_Motor_IsEnabled())
    {
        return false;
    }

    Motor_ApplyCommand(MOTOR_SIDE_RIGHT, command);
    return true;
}

bool BSP_Motor_Set(int16_t left_command, int16_t right_command)
{
    if (!BSP_Motor_IsEnabled())
    {
        return false;
    }

    Motor_ApplyCommand(MOTOR_SIDE_LEFT, left_command);
    Motor_ApplyCommand(MOTOR_SIDE_RIGHT, right_command);
    return true;
}

void BSP_Motor_BrakeLeft(void)
{
    if (s_initialized)
    {
        Motor_BrakeDirect(MOTOR_SIDE_LEFT);
    }
}

void BSP_Motor_BrakeRight(void)
{
    if (s_initialized)
    {
        Motor_BrakeDirect(MOTOR_SIDE_RIGHT);
    }
}

void BSP_Motor_BrakeAll(void)
{
    if (s_initialized)
    {
        Motor_BrakeDirect(MOTOR_SIDE_LEFT);
        Motor_BrakeDirect(MOTOR_SIDE_RIGHT);
    }
}
