#include "test_ball_balance.h"

#include "bsp_debug_uart.h"
#include "bsp_servo.h"
#include "stm32f4xx_hal.h"
#include "vision.h"

#include <limits.h>
#include <stdint.h>

/*
 * 视觉坐标系沿用main提交中的1280x960坐标。
 * 目标中心位置暂时保持为x=500。
 */
#define BALL_TARGET_CX                       ((int32_t)500)

/*
 * 当前机构实测控制范围：
 * 1600 us为水平稳定位置；
 * 1400 us和1800 us为钢球控制使用的安全限幅。
 */
#define BALL_SERVO_CENTER_US                 ((uint16_t)1600U)
#define BALL_SERVO_MIN_US                    ((uint16_t)1400U)
#define BALL_SERVO_MAX_US                    ((uint16_t)1800U)

/*
 * 推球比例系数使用Q10整数。
 * 1024表示1.0。
 */
#define BALL_PUSH_KP_Q10                     ((int32_t)1024)
#define BALL_GAIN_Q_SHIFT                    10

/*
 * 当 |error| < |speed| * 3 时开始反向制动。
 */
#define BALL_BRAKE_DISTANCE_GAIN             ((int32_t)3)

/*
 * 制动偏移：
 * brake = 60 + |speed| * 3 / 2
 * 最终限制到120 us。
 *
 * 修复main版本中BASE=200大于MAX=120的参数矛盾。
 */
#define BALL_BRAKE_BASE_US                   ((int32_t)60)
#define BALL_BRAKE_SPEED_NUMERATOR           ((int32_t)3)
#define BALL_BRAKE_SPEED_DENOMINATOR         ((int32_t)2)
#define BALL_BRAKE_MAX_US                    ((int32_t)120)

/* |error|不超过30像素时保持水平。 */
#define BALL_DEADZONE_PX                     ((int32_t)30)

/* 非零推球命令的最小舵机偏移。 */
#define BALL_MIN_STEP_US                     ((int32_t)20)

/* 调试日志周期。 */
#define BALL_PRINT_PERIOD_MS                 ((uint32_t)50U)
#define BALL_HEARTBEAT_MS                    ((uint32_t)2000U)

/* 连续丢失目标10帧后舵机回中。 */
#define BALL_LOST_TIMEOUT_FRAMES             ((uint32_t)10U)

static bool s_initialized = false;
static bool s_vision_ok = false;
static bool s_has_target = false;
static bool s_braking = false;
static bool s_vision_timeout_reported = false;

static uint16_t s_servo_pulse = BALL_SERVO_CENTER_US;

static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t s_lost_frames = 0U;
static uint32_t s_last_vision_sequence = 0U;

static int32_t s_last_cx = 0;
static int32_t s_last_error = 0;
static int32_t s_prev_cx = 0;
static int32_t s_speed = 0;
static int32_t s_last_delta = 0;

static int32_t BallBalance_AbsI32(int32_t value)
{
    if (value >= 0)
    {
        return value;
    }

    if (value == INT32_MIN)
    {
        return INT32_MAX;
    }

    return -value;
}

static int32_t BallBalance_Q10Multiply(int32_t gain_q10, int32_t value)
{
    int64_t product = (int64_t)gain_q10 * (int64_t)value;

    if (product >= 0)
    {
        product += (int64_t)(1L << (BALL_GAIN_Q_SHIFT - 1));
    }
    else
    {
        product -= (int64_t)(1L << (BALL_GAIN_Q_SHIFT - 1));
    }

    product >>= BALL_GAIN_Q_SHIFT;

    if (product > INT32_MAX)
    {
        return INT32_MAX;
    }

    if (product < INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)product;
}

static uint16_t BallBalance_ClampPulse(int32_t pulse_us)
{
    if (pulse_us < (int32_t)BALL_SERVO_MIN_US)
    {
        return BALL_SERVO_MIN_US;
    }

    if (pulse_us > (int32_t)BALL_SERVO_MAX_US)
    {
        return BALL_SERVO_MAX_US;
    }

    return (uint16_t)pulse_us;
}

static void BallBalance_ResetMotionEstimate(void)
{
    s_prev_cx = 0;
    s_speed = 0;
    s_last_delta = 0;
    s_braking = false;
}

static bool BallBalance_SetPulse(uint16_t pulse_us)
{
    if (pulse_us == s_servo_pulse)
    {
        return true;
    }

    if (!BSP_Servo_SetPulseUs(pulse_us))
    {
        return false;
    }

    s_servo_pulse = pulse_us;
    return true;
}

static void BallBalance_ReturnCenter(const char *reason)
{
    if (BallBalance_SetPulse(BALL_SERVO_CENTER_US))
    {
        (void)BSP_Debug_Printf(
            "BALL_BAL,%s,PULSE=%u\r\n",
            reason,
            (unsigned int)BALL_SERVO_CENTER_US);
    }

    BallBalance_ResetMotionEstimate();
}

static int32_t BallBalance_CalculateControlDelta(void)
{
    int32_t error_abs = BallBalance_AbsI32(s_last_error);
    int32_t speed_abs = BallBalance_AbsI32(s_speed);
    bool moving_to_target;

    moving_to_target =
        ((s_last_error > 0) && (s_speed > 0)) ||
        ((s_last_error < 0) && (s_speed < 0));

    if (moving_to_target &&
        (error_abs < (speed_abs * BALL_BRAKE_DISTANCE_GAIN)))
    {
        int32_t brake =
            BALL_BRAKE_BASE_US +
            ((speed_abs * BALL_BRAKE_SPEED_NUMERATOR) /
             BALL_BRAKE_SPEED_DENOMINATOR);

        if (brake > BALL_BRAKE_MAX_US)
        {
            brake = BALL_BRAKE_MAX_US;
        }

        s_braking = true;

        /*
         * 钢球向右运动时向左制动；
         * 钢球向左运动时向右制动。
         */
        return (s_speed > 0) ? -brake : brake;
    }

    s_braking = false;
    return BallBalance_Q10Multiply(
        BALL_PUSH_KP_Q10,
        s_last_error);
}

static void BallBalance_ProcessFoundTarget(uint16_t center_x)
{
    int32_t delta;
    int32_t raw_pulse;
    uint16_t clamped_pulse;

    s_last_cx = (int32_t)center_x;
    s_last_error = BALL_TARGET_CX - s_last_cx;
    s_has_target = true;
    s_lost_frames = 0U;
    s_vision_timeout_reported = false;

    if (s_prev_cx != 0)
    {
        /* 正值表示钢球向图像右侧移动。 */
        s_speed = s_last_cx - s_prev_cx;
    }
    else
    {
        s_speed = 0;
    }
    s_prev_cx = s_last_cx;

    if (BallBalance_AbsI32(s_last_error) <= BALL_DEADZONE_PX)
    {
        s_last_delta = 0;
        s_braking = false;
        (void)BallBalance_SetPulse(BALL_SERVO_CENTER_US);
        return;
    }

    delta = BallBalance_CalculateControlDelta();

    if ((delta > 0) && (delta < BALL_MIN_STEP_US))
    {
        delta = BALL_MIN_STEP_US;
    }
    else if ((delta < 0) && (delta > -BALL_MIN_STEP_US))
    {
        delta = -BALL_MIN_STEP_US;
    }

    s_last_delta = delta;
    raw_pulse = (int32_t)BALL_SERVO_CENTER_US + delta;
    clamped_pulse = BallBalance_ClampPulse(raw_pulse);
    (void)BallBalance_SetPulse(clamped_pulse);
}

static void BallBalance_ProcessNoTarget(void)
{
    s_has_target = false;
    s_lost_frames++;
    BallBalance_ResetMotionEstimate();

    if (s_lost_frames >= BALL_LOST_TIMEOUT_FRAMES)
    {
        BallBalance_ReturnCenter("LOST_RETURN_CENTER");
        s_lost_frames = 0U;
    }
}

bool Test_BallBalance_Init(void)
{
    uint32_t now_ms;

    s_initialized = false;
    s_vision_ok = false;
    s_has_target = false;
    s_braking = false;
    s_vision_timeout_reported = false;

    s_servo_pulse = BALL_SERVO_CENTER_US;

    s_lost_frames = 0U;
    s_last_vision_sequence = 0U;

    s_last_cx = 0;
    s_last_error = 0;
    BallBalance_ResetMotionEstimate();

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    (void)BSP_Debug_Printf(
        "BALL_BAL,START,TARGET=%ld,DEADZONE=%ld,"
        "KP_Q10=%ld,BRAKE_BASE=%ld,BRAKE_MAX=%ld\r\n",
        (long)BALL_TARGET_CX,
        (long)BALL_DEADZONE_PX,
        (long)BALL_PUSH_KP_Q10,
        (long)BALL_BRAKE_BASE_US,
        (long)BALL_BRAKE_MAX_US);

    if (!BSP_Servo_Init())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_INIT\r\n");
        return false;
    }

    if (!BSP_Servo_SetPulseUs(BALL_SERVO_CENTER_US))
    {
        (void)BSP_Debug_Printf("ERR,SERVO_SET_CENTER\r\n");
        return false;
    }

    if (!BSP_Servo_Enable())
    {
        (void)BSP_Debug_Printf("ERR,SERVO_ENABLE\r\n");
        return false;
    }

    s_servo_pulse = BALL_SERVO_CENTER_US;

    (void)BSP_Debug_Printf(
        "OK,SERVO,PULSE_US=%u,EN=%u\r\n",
        (unsigned int)BSP_Servo_GetPulseUs(),
        BSP_Servo_IsEnabled() ? 1U : 0U);

    /*
     * 使用follow底版当前的Vision模块：
     * BSP只收字节，Vision模块负责协议解析。
     *
     * 视觉初始化失败保持为非致命错误：
     * 舵机保持水平，便于单独检查舵机和串口。
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

    now_ms = HAL_GetTick();
    s_last_print_ms = now_ms;
    s_last_heartbeat_ms = now_ms;
    s_initialized = true;

    return true;
}

void Test_BallBalance_Update(void)
{
    VisionStatus_t vision_status;
    bool have_vision_status = false;
    uint32_t now_ms;

    if (!s_initialized)
    {
        return;
    }

    BSP_DebugUart_Process();

    if (s_vision_ok)
    {
        Vision_Update();
        have_vision_status =
            Vision_GetStatus(&vision_status);
    }

    now_ms = HAL_GetTick();

    if (s_vision_ok && have_vision_status)
    {
        if (vision_status.has_frame &&
            (vision_status.sequence != s_last_vision_sequence))
        {
            s_last_vision_sequence = vision_status.sequence;

            if (vision_status.frame.found)
            {
                BallBalance_ProcessFoundTarget(
                    vision_status.frame.center_x);
            }
            else
            {
                BallBalance_ProcessNoTarget();
            }
        }

        /*
         * 串口断开、协议停止输出或最后一帧超过100 ms：
         * 不再沿用旧位置和旧速度，舵机回到水平位置。
         */
        if (vision_status.has_frame &&
            !vision_status.data_valid)
        {
            s_has_target = false;
            s_lost_frames = 0U;

            if (!s_vision_timeout_reported)
            {
                BallBalance_ReturnCenter(
                    "VISION_TIMEOUT_RETURN_CENTER");
                s_vision_timeout_reported = true;
            }
        }
    }

    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >=
        BALL_HEARTBEAT_MS)
    {
        s_last_heartbeat_ms = now_ms;

        if (s_vision_ok && have_vision_status)
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,HB,SEQ=%lu,VF=%lu,IF=%lu,"
                "PO=%lu,UE=%lu,VALID=%u\r\n",
                (unsigned long)vision_status.sequence,
                (unsigned long)vision_status.valid_frame_count,
                (unsigned long)vision_status.invalid_frame_count,
                (unsigned long)vision_status.protocol_overflow_count,
                (unsigned long)vision_status.uart_error_count,
                vision_status.data_valid ? 1U : 0U);
        }
        else
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,HB,VISION=0,PULSE=%u\r\n",
                (unsigned int)s_servo_pulse);
        }
    }

    if ((uint32_t)(now_ms - s_last_print_ms) >=
        BALL_PRINT_PERIOD_MS)
    {
        s_last_print_ms = now_ms;

        if (s_has_target)
        {
            (void)BSP_Debug_Printf(
                "BALL_BAL,CX=%ld,ERR=%ld,SPD=%ld,"
                "D=%ld,%s,PULSE=%u\r\n",
                (long)s_last_cx,
                (long)s_last_error,
                (long)s_speed,
                (long)s_last_delta,
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

    (void)BSP_Servo_SetPulseUs(BALL_SERVO_CENTER_US);
    s_servo_pulse = BALL_SERVO_CENTER_US;

    if (s_vision_ok)
    {
        Vision_Stop();
    }

    BSP_Servo_Disable();

    s_vision_ok = false;
    s_initialized = false;

    (void)BSP_Debug_Printf(
        "BALL_BAL,STOP,PULSE=%u\r\n",
        (unsigned int)s_servo_pulse);
}

bool Test_BallBalance_IsInitialized(void)
{
    return s_initialized;
}
