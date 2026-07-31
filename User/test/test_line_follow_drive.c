#include "test_line_follow_drive.h"

#include "bsp_debug_uart.h"
#include "chassis.h"
#include "line_follow.h"
#include "line_follow_control.h"
#include "line_follow_control_config.h"
#include "line_sensor.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

typedef struct
{
    GPIO_PinState raw_state;
    GPIO_PinState stable_state;
    uint32_t raw_change_ms;
} TestLineFollowKey_t;

static bool s_initialized = false;
static uint32_t s_last_report_ms = 0U;

static LineFollowResult_t s_latest_line_result;
static TestLineFollowKey_t s_key;

static void Test_LineFollowDrive_InitKey(void)
{
    GPIO_PinState current =
        HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);

    s_key.raw_state = current;
    s_key.stable_state = current;
    s_key.raw_change_ms = HAL_GetTick();
}

static bool Test_LineFollowDrive_KeyPressed(uint32_t now_ms)
{
    GPIO_PinState raw =
        HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);

    if (raw != s_key.raw_state)
    {
        s_key.raw_state = raw;
        s_key.raw_change_ms = now_ms;
    }

    if ((raw != s_key.stable_state) &&
        ((uint32_t)(now_ms - s_key.raw_change_ms) >=
         LINE_FOLLOW_DRIVE_KEY_DEBOUNCE_MS))
    {
        s_key.stable_state = raw;

        if (s_key.stable_state == GPIO_PIN_SET)
        {
            return true;
        }
    }

    return false;
}

static void Test_LineFollowDrive_ToggleRun(void)
{
    LineFollowControlStatus_t control;

    if (!LineFollowControl_GetStatus(&control))
    {
        return;
    }

    if (LineFollowControl_IsRunning())
    {
        if (!LineFollowControl_RequestStop(
                LINE_FOLLOW_CONTROL_STOP_USER))
        {
            (void)BSP_Debug_Printf(
                "ERR,LINE_FOLLOW_FAST_STOP\r\n");
            return;
        }

        (void)BSP_Debug_Printf(
            "LFD,EVENT=KEY_FAST_STOP\r\n");
        return;
    }

    if (LineFollowControl_IsStopping())
    {
        LineFollowControl_Stop(
            LINE_FOLLOW_CONTROL_STOP_USER);

        (void)BSP_Debug_Printf(
            "LFD,EVENT=KEY_EMERGENCY_STOP\r\n");
        return;
    }

    if (!LineFollowControl_Start())
    {
        (void)BSP_Debug_Printf(
            "ERR,LINE_FOLLOW_CONTROL_START\r\n");
        return;
    }

    (void)BSP_Debug_Printf(
        "LFD,EVENT=KEY_START\r\n");
}

static void Test_LineFollowDrive_PrintStatus(void)
{
    LineFollowControlStatus_t control;
    ChassisStatus_t chassis;

    if (!LineFollowControl_GetStatus(&control))
    {
        return;
    }

    (void)BSP_Debug_Printf(
        "LFD,R=%u,SP=%u,M=%s,X=%s,LS=%s,MASK=0x%02X,"
        "E=%d,DE=%d,B=%ld,C=%ld,T=%ld/%ld,AGE=%lu\r\n",
        control.running ? 1U : 0U,
        control.stopping ? 1U : 0U,
        LineFollowControl_ModeName(control.mode),
        LineFollowControl_StopReasonName(
            control.stop_reason),
        LineFollow_StateName(control.line_state),
        (unsigned int)control.black_mask,
        (int)control.error,
        (int)control.error_delta,
        (long)control.base_speed_mm_s,
        (long)control.correction_mm_s,
        (long)control.left_target_mm_s,
        (long)control.right_target_mm_s,
        (unsigned long)control.state_elapsed_ms);

    if (Chassis_GetStatus(&chassis))
    {
        (void)BSP_Debug_Printf(
            "LFC,S=%lu,MODE=%s,CMD=%ld/%ld,RMP=%ld/%ld,"
            "MEA=%ld/%ld,F=%ld,T=%ld,AF=%ld,AT=%ld,"
            "PWM=%d/%d,A=%u,STOP=%u,V=%u,OVR=%lu\r\n",
            (unsigned long)chassis.control_sequence,
            Chassis_MotionModeName(chassis.motion_mode),
            (long)chassis.left_command_mm_s,
            (long)chassis.right_command_mm_s,
            (long)chassis.left_target_mm_s,
            (long)chassis.right_target_mm_s,
            (long)chassis.left_measured_mm_s,
            (long)chassis.right_measured_mm_s,
            (long)chassis.forward_target_mm_s,
            (long)chassis.turn_target_mm_s,
            (long)chassis.forward_accel_mm_s2,
            (long)chassis.turn_accel_mm_s2,
            (int)chassis.left_pwm,
            (int)chassis.right_pwm,
            chassis.speed_ramp_active ? 1U : 0U,
            chassis.motion_stopped ? 1U : 0U,
            chassis.encoder_sample_valid ? 1U : 0U,
            (unsigned long)chassis.timing_overrun_count);
    }
}

bool Test_LineFollowDrive_Init(void)
{
    s_initialized = false;

    if (!BSP_DebugUart_Init())
    {
        return false;
    }

    if (!LineSensor_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,LINE_SENSOR_INIT\r\n");
        return false;
    }

    if (!LineSensor_IsBackendConfigConfirmed())
    {
        (void)BSP_Debug_Printf(
            "ERR,LINE_SENSOR_CONFIG_NOT_CONFIRMED\r\n");
        (void)LineSensor_Stop();
        return false;
    }

    LineFollow_Init();

    if (!LineFollowControl_Init())
    {
        (void)BSP_Debug_Printf(
            "ERR,LINE_FOLLOW_CONTROL_INIT\r\n");
        (void)LineSensor_Stop();
        return false;
    }

    Test_LineFollowDrive_InitKey();

    s_last_report_ms = HAL_GetTick();
    s_initialized = true;

    (void)BSP_Debug_Printf(
        "TEST,LINE_FOLLOW_DRIVE,READY,BACKEND=%s,"
        "KEY=KEY0_PE4_ACTIVE_HIGH,DEBOUNCE_MS=%lu\r\n",
        LineSensor_GetBackendName(),
        (unsigned long)LINE_FOLLOW_DRIVE_KEY_DEBOUNCE_MS);

    (void)BSP_Debug_Printf(
        "LFD,CFG=BASE_360_TO_120,MAX=500,"
        "KP_Q10=%ld,KD_Q10=%ld,LOST=%lu,BLACK=%lu,"
        "REVERSE=0,PROFILE=JERK_LIMITED\r\n",
        (long)LINE_FOLLOW_CONTROL_KP_Q10,
        (long)LINE_FOLLOW_CONTROL_KD_Q10,
        (unsigned long)LINE_FOLLOW_CONTROL_LOST_TIMEOUT_MS,
        (unsigned long)LINE_FOLLOW_CONTROL_ALL_BLACK_TIMEOUT_MS);

    return true;
}

void Test_LineFollowDrive_Update(void)
{
    LineSensorFrame_t frame;
    uint32_t now_ms;

    BSP_DebugUart_Process();

    if (!s_initialized)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if (Test_LineFollowDrive_KeyPressed(now_ms))
    {
        Test_LineFollowDrive_ToggleRun();
    }

    if (LineSensor_Update() &&
        LineSensor_GetFrame(&frame))
    {
        if (LineFollow_Update(&frame) &&
            LineFollow_GetResult(&s_latest_line_result))
        {
            (void)LineFollowControl_Submit(
                &s_latest_line_result);
        }
    }

    LineFollowControl_Process();

    if ((uint32_t)(now_ms - s_last_report_ms) >=
        LINE_FOLLOW_DRIVE_REPORT_PERIOD_MS)
    {
        s_last_report_ms = now_ms;
        Test_LineFollowDrive_PrintStatus();
    }
}

void Test_LineFollowDrive_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    LineFollowControl_Shutdown();
    (void)LineSensor_Stop();

    s_initialized = false;

    (void)BSP_Debug_Printf(
        "TEST,LINE_FOLLOW_DRIVE,STOP\r\n");
}

bool Test_LineFollowDrive_IsInitialized(void)
{
    return s_initialized;
}
