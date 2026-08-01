/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_debug_uart.h"
#include "bsp_line_adc.h"
#include "bsp_line_uart.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_vision_uart.h"
#include "chassis.h"
#include "main_app.h"
#include "project_servo_level.h"
#include "task4_main_ball.h"
#include "test_runner.h"

#include <limits.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROJECT_ENTRY_MAIN_APP      1U
#define PROJECT_ENTRY_TEST_RUNNER   2U
#define PROJECT_ENTRY_MODE          PROJECT_ENTRY_MAIN_APP

/* Task 4 ball controller: implementation intentionally lives in main.c. */
#define T4_BALL_TARGET_CX                         ((int32_t)653)
#define T4_BALL_ONE_CM_PX                         ((int32_t)43)
#define T4_BALL_ONE_CM_MM                         ((int32_t)10)
#define T4_BALL_CAPTURE_REQUIRED_FRAMES           ((uint8_t)3U)
#define T4_BALL_CAPTURE_MAX_DELTA_PX              ((int32_t)12)
#define T4_BALL_CAPTURE_MIN_MM                    ((int32_t)-125)
#define T4_BALL_CAPTURE_MAX_MM                    ((int32_t)125)
#define T4_BALL_SERVO_CENTER_US                   ((int32_t)PROJECT_SERVO_HORIZONTAL_US)

/* Preserve V3 relative authority after shifting center 1700 -> 1650 us. */
#define T4_BALL_SERVO_MIN_US                      ((int32_t)1270)
#define T4_BALL_SERVO_MAX_US                      ((int32_t)1950)
#define T4_BALL_CONTROL_NEG_LIMIT_US              ((int32_t)-380)
#define T4_BALL_CONTROL_POS_LIMIT_US              ((int32_t)300)

/* Position prediction and gain scheduling. */
#define T4_BALL_POSITION_DEADBAND_PX              ((int32_t)3)
#define T4_BALL_POSITION_NEAR_PX                  ((int32_t)12)
#define T4_BALL_POSITION_MID_PX                   ((int32_t)28)
#define T4_BALL_PREDICT_NORMAL_MS                 ((int32_t)90)
#define T4_BALL_PREDICT_DYNAMIC_MS                ((int32_t)130)
#define T4_BALL_PREDICT_LIMIT_PX                  ((int32_t)90)

/* Time-normalized velocity damping (gain numerator / 10). */
#define T4_BALL_VELOCITY_GAIN_NORMAL_NUM          ((int32_t)4)
#define T4_BALL_VELOCITY_GAIN_DYNAMIC_NUM         ((int32_t)5)
#define T4_BALL_RAW_VELOCITY_GAIN_NUM             ((int32_t)1)
#define T4_BALL_GAIN_DENOMINATOR                   ((int32_t)10)
#define T4_BALL_RAW_VELOCITY_LIMIT_PX_S           ((int32_t)1600)

/* Static tilt learning is frozen during vehicle acceleration/turning. */
#define T4_BALL_BIAS_LIMIT_US                     ((int32_t)100)
#define T4_BALL_BIAS_ADAPT_MIN_ERROR_PX           ((int32_t)5)
#define T4_BALL_BIAS_ADAPT_MAX_ERROR_PX           ((int32_t)24)
#define T4_BALL_BIAS_ADAPT_MAX_VELOCITY_PX_S      ((int32_t)35)
#define T4_BALL_BIAS_ADAPT_DIVIDER                ((uint8_t)3U)

/* Vehicle acceleration feedforward. Flip SIGN only if the real direction is wrong. */
#define T4_BALL_ACCEL_FF_SIGN                      ((int32_t)-1)
#define T4_BALL_PLANNED_ACCEL_FF_DIVISOR           ((int32_t)5)
#define T4_BALL_MEASURED_ACCEL_FF_DIVISOR          ((int32_t)12)
#define T4_BALL_ACCEL_FF_LIMIT_US                  ((int32_t)150)
#define T4_BALL_ACCEL_FF_STEP_US                   ((int32_t)45)
#define T4_BALL_MEASURED_ACCEL_LIMIT_MM_S2         ((int32_t)3000)

/*
 * Launch-only feedforward. The real car consistently throws the ball toward
 * +X during launch, so launch compensation must command the same direction
 * as the negative position correction used for a positive ball error.
 */
#define T4_BALL_LAUNCH_ACCEL_MIN_MM_S2             ((int32_t)35)
#define T4_BALL_LAUNCH_STRONG_SPEED_MM_S           ((int32_t)80)
#define T4_BALL_LAUNCH_MEDIUM_SPEED_MM_S           ((int32_t)180)
#define T4_BALL_LAUNCH_SPEED_END_MM_S              ((int32_t)240)
#define T4_BALL_LAUNCH_FF_STRONG_US                 ((int32_t)140)
#define T4_BALL_LAUNCH_FF_MEDIUM_US                 ((int32_t)100)
#define T4_BALL_LAUNCH_FF_LIMIT_US                  ((int32_t)180)
#define T4_BALL_LAUNCH_FF_STEP_STRONG_US            ((int32_t)90)
#define T4_BALL_LAUNCH_FF_STEP_MEDIUM_US            ((int32_t)70)
#define T4_BALL_LAUNCH_TIMEOUT_MS                   ((uint32_t)900U)

/* Gain scheduling from chassis state. */
#define T4_BALL_DYNAMIC_ACCEL_MM_S2                ((int32_t)100)
#define T4_BALL_DYNAMIC_TURN_MM_S                  ((int32_t)90)
#define T4_BALL_DYNAMIC_TURN_ACCEL_MM_S2           ((int32_t)350)
#define T4_BALL_DYNAMIC_SPEED_MM_S                 ((int32_t)260)

/* Servo slew is intentionally much faster only for real recovery. */
#define T4_BALL_SLEW_HOLD_US                       ((int32_t)24)
#define T4_BALL_SLEW_NORMAL_US                     ((int32_t)34)
#define T4_BALL_SLEW_FAST_US                       ((int32_t)62)
#define T4_BALL_SLEW_RECOVER_US                    ((int32_t)100)
#define T4_BALL_FAST_ERROR_PX                      ((int32_t)18)
#define T4_BALL_RECOVER_ERROR_PX                   ((int32_t)34)
#define T4_BALL_FAST_VELOCITY_PX_S                 ((int32_t)150)
#define T4_BALL_RECOVER_VELOCITY_PX_S              ((int32_t)320)

/*
 * Target-position gain scheduling.
 * Center target (Task 4/5, 0 mm) retains the original controller exactly.
 * From 20 mm away from center the controller is progressively damped; at
 * 60 mm and beyond the full off-center profile is active. This compensates
 * for non-uniform rod/servo authority and optical sensitivity at arbitrary
 * Task 6 balance positions without widening the +/-1 cm judging threshold.
 */
#define T4_BALL_TARGET_SCHEDULE_START_MM            ((int32_t)20)
#define T4_BALL_TARGET_SCHEDULE_FULL_MM             ((int32_t)60)
#define T4_BALL_SCHEDULE_ONE_MILLI                   ((int32_t)1000)
#define T4_BALL_OFFCENTER_POSITION_SCALE_MILLI      ((int32_t)650)
#define T4_BALL_OFFCENTER_VELOCITY_SCALE_MILLI      ((int32_t)1650)
#define T4_BALL_OFFCENTER_RAW_VEL_SCALE_MILLI       ((int32_t)500)
#define T4_BALL_OFFCENTER_PREDICT_SCALE_MILLI       ((int32_t)700)
#define T4_BALL_OFFCENTER_SLEW_SCALE_MILLI          ((int32_t)600)
#define T4_BALL_OFFCENTER_RECOVER_SLEW_MILLI        ((int32_t)750)
#define T4_BALL_OFFCENTER_DEADBAND_EXTRA_PX         ((int32_t)3)
#define T4_BALL_OFFCENTER_BIAS_DIVIDER_EXTRA        ((int32_t)4)

/*
 * The mechanism becomes still more sensitive near either end of the rod.
 * Continue scheduling from 60 to 110 mm instead of freezing the -60 mm
 * profile for every farther target.
 */
#define T4_BALL_EXTREME_SCHEDULE_START_MM           ((int32_t)60)
#define T4_BALL_EXTREME_SCHEDULE_FULL_MM            ((int32_t)110)
#define T4_BALL_EXTREME_POSITION_SCALE_MILLI        ((int32_t)500)
#define T4_BALL_EXTREME_VELOCITY_SCALE_MILLI        ((int32_t)1900)
#define T4_BALL_EXTREME_RAW_VEL_SCALE_MILLI         ((int32_t)350)
#define T4_BALL_EXTREME_PREDICT_SCALE_MILLI         ((int32_t)550)
#define T4_BALL_EXTREME_SLEW_SCALE_MILLI            ((int32_t)420)
#define T4_BALL_EXTREME_RECOVER_SLEW_MILLI          ((int32_t)600)
#define T4_BALL_EXTREME_DEADBAND_EXTRA_PX           ((int32_t)5)
#define T4_BALL_EXTREME_BIAS_DIVIDER_EXTRA          ((int32_t)8)
#define T4_BALL_EXTREME_FAST_ERROR_EXTRA_PX         ((int32_t)10)
#define T4_BALL_EXTREME_RECOVER_ERROR_EXTRA_PX      ((int32_t)24)
#define T4_BALL_EXTREME_FAST_VELOCITY_EXTRA_PX_S    ((int32_t)90)
#define T4_BALL_EXTREME_RECOVER_VELOCITY_EXTRA_PX_S ((int32_t)170)

/*
 * After the ball crosses a far-end target, brake it for several vision frames
 * before applying the opposite position push. This removes the repeated
 * up/down servo reversal that otherwise sustains a limit-cycle oscillation.
 */
#define T4_BALL_CROSS_BRAKE_FRAMES                  ((uint8_t)3U)
#define T4_BALL_CROSS_BRAKE_MAX_ERROR_PX            ((int32_t)58)
#define T4_BALL_CROSS_POSITION_SCALE_MILLI          ((int32_t)0)
#define T4_BALL_CROSS_DAMPING_SCALE_MILLI           ((int32_t)1250)
#define T4_BALL_CROSS_SLEW_US                       ((int32_t)8)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static bool s_t4_ball_initialized = false;
static uint32_t s_t4_ball_last_vision_sequence = 0U;
static uint32_t s_t4_ball_last_vision_timestamp_ms = 0U;
static uint32_t s_t4_ball_last_chassis_timestamp_ms = 0U;
static int32_t s_t4_ball_previous_center_x = 0;
static int32_t s_t4_ball_previous_forward_speed_mm_s = 0;
static int32_t s_t4_ball_filtered_velocity_px_s = 0;
static int32_t s_t4_ball_filtered_measured_accel_mm_s2 = 0;
static int32_t s_t4_ball_filtered_feedforward_us = 0;
static uint8_t s_t4_ball_bias_divider = 0U;
static int8_t s_t4_ball_last_error_sign = 0;
static uint8_t s_t4_ball_cross_brake_frames = 0U;
static bool s_t4_ball_launch_active = false;
static uint32_t s_t4_ball_launch_start_ms = 0U;
static uint8_t s_t4_ball_capture_count = 0U;
static int32_t s_t4_ball_capture_last_x = 0;
static int32_t s_t4_ball_capture_sum_x = 0;
static int32_t s_t4_ball_capture_sum_mm = 0;
static ChassisStatus_t s_t4_ball_chassis_status;
static Task4MainBallStatus_t s_t4_ball_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool Project_ServoHoldHorizontal(void)
{
    if ((!BSP_Servo_IsInitialized()) && (!BSP_Servo_Init()))
    {
        return false;
    }

    if (!BSP_Servo_SetPulseUs(PROJECT_SERVO_HORIZONTAL_US))
    {
        return false;
    }

    return BSP_Servo_Enable();
}

static int32_t T4Ball_AbsI32(int32_t value)
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

static int32_t T4Ball_ClampI32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int32_t T4Ball_ApproachI32(int32_t current,
                                  int32_t target,
                                  int32_t maximum_step)
{
    int32_t difference = target - current;

    if (difference > maximum_step)
    {
        return current + maximum_step;
    }
    if (difference < -maximum_step)
    {
        return current - maximum_step;
    }
    return target;
}

static int32_t T4Ball_MulDivI32(int32_t value,
                                int32_t numerator,
                                int32_t denominator)
{
    int64_t product;

    if (denominator == 0)
    {
        return 0;
    }
    product = (int64_t)value * (int64_t)numerator;
    if (product >= 0)
    {
        product += denominator / 2;
    }
    else
    {
        product -= denominator / 2;
    }
    product /= denominator;
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

static int32_t T4Ball_TargetScheduleMilli(void)
{
    int32_t offset_mm = T4Ball_AbsI32(s_t4_ball_status.target_mm);
    int32_t span_mm = T4_BALL_TARGET_SCHEDULE_FULL_MM -
                      T4_BALL_TARGET_SCHEDULE_START_MM;

    if (offset_mm <= T4_BALL_TARGET_SCHEDULE_START_MM)
    {
        return 0;
    }
    if ((offset_mm >= T4_BALL_TARGET_SCHEDULE_FULL_MM) ||
        (span_mm <= 0))
    {
        return T4_BALL_SCHEDULE_ONE_MILLI;
    }
    return T4Ball_MulDivI32(
        offset_mm - T4_BALL_TARGET_SCHEDULE_START_MM,
        T4_BALL_SCHEDULE_ONE_MILLI,
        span_mm);
}

static int32_t T4Ball_ExtremeScheduleMilli(void)
{
    int32_t offset_mm = T4Ball_AbsI32(s_t4_ball_status.target_mm);
    int32_t span_mm = T4_BALL_EXTREME_SCHEDULE_FULL_MM -
                      T4_BALL_EXTREME_SCHEDULE_START_MM;

    if (offset_mm <= T4_BALL_EXTREME_SCHEDULE_START_MM)
    {
        return 0;
    }
    if ((offset_mm >= T4_BALL_EXTREME_SCHEDULE_FULL_MM) ||
        (span_mm <= 0))
    {
        return T4_BALL_SCHEDULE_ONE_MILLI;
    }
    return T4Ball_MulDivI32(
        offset_mm - T4_BALL_EXTREME_SCHEDULE_START_MM,
        T4_BALL_SCHEDULE_ONE_MILLI,
        span_mm);
}

static int32_t T4Ball_ScheduledValue(int32_t center_value,
                                     int32_t offcenter_value,
                                     int32_t schedule_milli)
{
    return center_value + T4Ball_MulDivI32(
        offcenter_value - center_value,
        schedule_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
}

static int32_t T4Ball_ScheduledSlew(int32_t base_slew_us,
                                    int32_t schedule_milli,
                                    bool recovery)
{
    int32_t extreme_schedule_milli = T4Ball_ExtremeScheduleMilli();
    int32_t scale_milli = T4Ball_ScheduledValue(
        T4_BALL_SCHEDULE_ONE_MILLI,
        recovery ? T4_BALL_OFFCENTER_RECOVER_SLEW_MILLI :
                   T4_BALL_OFFCENTER_SLEW_SCALE_MILLI,
        schedule_milli);
    int32_t scheduled;

    scale_milli = T4Ball_ScheduledValue(
        scale_milli,
        recovery ? T4_BALL_EXTREME_RECOVER_SLEW_MILLI :
                   T4_BALL_EXTREME_SLEW_SCALE_MILLI,
        extreme_schedule_milli);
    scheduled = T4Ball_MulDivI32(
        base_slew_us,
        scale_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);

    return (scheduled > 0) ? scheduled : 1;
}

static bool T4Ball_WritePulse(int32_t desired_pulse_us,
                              int32_t maximum_step_us)
{
    int32_t current_pulse_us = (int32_t)s_t4_ball_status.servo_pulse_us;
    int32_t delta_us;

    desired_pulse_us = T4Ball_ClampI32(
        desired_pulse_us,
        T4_BALL_SERVO_MIN_US,
        T4_BALL_SERVO_MAX_US);

    delta_us = desired_pulse_us - current_pulse_us;
    delta_us = T4Ball_ClampI32(
        delta_us,
        -maximum_step_us,
        maximum_step_us);
    current_pulse_us += delta_us;

    if (!BSP_Servo_SetPulseUs((uint16_t)current_pulse_us))
    {
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_SERVO_ERROR;
        return false;
    }

    s_t4_ball_status.servo_pulse_us = (uint16_t)current_pulse_us;
    s_t4_ball_status.servo_step_limit_us = (uint16_t)maximum_step_us;
    return true;
}

static bool T4Ball_UpdateVehicleMotion(void)
{
    int32_t measured_forward_speed_mm_s;
    int32_t measured_accel_mm_s2 = 0;
    int32_t feedforward_target_us;
    int32_t feedforward_step_us;
    int32_t feedforward_limit_us;
    int32_t abs_planned_accel_mm_s2;
    int32_t abs_forward_speed_mm_s;
    int32_t launch_floor_us;
    uint32_t dt_ms;

    uint32_t previous_timestamp_ms =
        s_t4_ball_last_chassis_timestamp_ms;

    if (!Chassis_GetStatus(&s_t4_ball_chassis_status))
    {
        return false;
    }
    if ((previous_timestamp_ms != 0U) &&
        (s_t4_ball_chassis_status.timestamp_ms ==
         previous_timestamp_ms))
    {
        return false;
    }

    measured_forward_speed_mm_s =
        (s_t4_ball_chassis_status.left_measured_mm_s +
         s_t4_ball_chassis_status.right_measured_mm_s) / 2;

    if ((s_t4_ball_last_chassis_timestamp_ms != 0U) &&
        (s_t4_ball_chassis_status.timestamp_ms >
         s_t4_ball_last_chassis_timestamp_ms))
    {
        dt_ms = s_t4_ball_chassis_status.timestamp_ms -
                s_t4_ball_last_chassis_timestamp_ms;
        if ((dt_ms >= 2U) && (dt_ms <= 100U))
        {
            measured_accel_mm_s2 = (int32_t)(
                ((int64_t)(measured_forward_speed_mm_s -
                           s_t4_ball_previous_forward_speed_mm_s) * 1000LL) /
                (int64_t)dt_ms);
            measured_accel_mm_s2 = T4Ball_ClampI32(
                measured_accel_mm_s2,
                -T4_BALL_MEASURED_ACCEL_LIMIT_MM_S2,
                T4_BALL_MEASURED_ACCEL_LIMIT_MM_S2);
        }
    }

    s_t4_ball_last_chassis_timestamp_ms =
        s_t4_ball_chassis_status.timestamp_ms;
    s_t4_ball_previous_forward_speed_mm_s =
        measured_forward_speed_mm_s;
    s_t4_ball_filtered_measured_accel_mm_s2 =
        (s_t4_ball_filtered_measured_accel_mm_s2 * 3 +
         measured_accel_mm_s2) / 4;

    feedforward_target_us = T4_BALL_ACCEL_FF_SIGN *
        (s_t4_ball_chassis_status.forward_accel_mm_s2 /
             T4_BALL_PLANNED_ACCEL_FF_DIVISOR +
         s_t4_ball_filtered_measured_accel_mm_s2 /
             T4_BALL_MEASURED_ACCEL_FF_DIVISOR);
    feedforward_step_us = T4_BALL_ACCEL_FF_STEP_US;
    feedforward_limit_us = T4_BALL_ACCEL_FF_LIMIT_US;

    abs_planned_accel_mm_s2 = T4Ball_AbsI32(
        s_t4_ball_chassis_status.forward_accel_mm_s2);
    abs_forward_speed_mm_s = T4Ball_AbsI32(measured_forward_speed_mm_s);
    if (s_t4_ball_launch_active)
    {
        if ((s_t4_ball_launch_start_ms == 0U) &&
            ((abs_planned_accel_mm_s2 >=
              T4_BALL_LAUNCH_ACCEL_MIN_MM_S2) ||
             (T4Ball_AbsI32(measured_forward_speed_mm_s) >= 20)))
        {
            s_t4_ball_launch_start_ms =
                s_t4_ball_chassis_status.timestamp_ms;
        }

        if ((s_t4_ball_launch_start_ms != 0U) &&
            (((uint32_t)(s_t4_ball_chassis_status.timestamp_ms -
                         s_t4_ball_launch_start_ms) >=
              T4_BALL_LAUNCH_TIMEOUT_MS) ||
             (T4Ball_AbsI32(measured_forward_speed_mm_s) >=
              T4_BALL_LAUNCH_SPEED_END_MM_S)))
        {
            s_t4_ball_launch_active = false;
        }
        else if ((s_t4_ball_chassis_status.forward_accel_mm_s2 > 0) ||
                 (measured_accel_mm_s2 > 0))
        {
            /*
             * Strong only at the first launch impulse, then taper with speed
             * so the compensation does not create a negative rebound.
             */
            launch_floor_us = 0;
            if (abs_forward_speed_mm_s <
                T4_BALL_LAUNCH_STRONG_SPEED_MM_S)
            {
                launch_floor_us = T4_BALL_LAUNCH_FF_STRONG_US;
                feedforward_step_us = T4_BALL_LAUNCH_FF_STEP_STRONG_US;
            }
            else if (abs_forward_speed_mm_s <
                     T4_BALL_LAUNCH_MEDIUM_SPEED_MM_S)
            {
                launch_floor_us = T4_BALL_LAUNCH_FF_MEDIUM_US;
                feedforward_step_us = T4_BALL_LAUNCH_FF_STEP_MEDIUM_US;
            }

            if ((launch_floor_us > 0) &&
                (feedforward_target_us > -launch_floor_us))
            {
                feedforward_target_us = -launch_floor_us;
            }
            feedforward_target_us = T4Ball_ClampI32(
                feedforward_target_us,
                -T4_BALL_LAUNCH_FF_LIMIT_US,
                T4_BALL_LAUNCH_FF_LIMIT_US);
            feedforward_limit_us = T4_BALL_LAUNCH_FF_LIMIT_US;
        }
    }

    feedforward_target_us = T4Ball_ClampI32(
        feedforward_target_us,
        -feedforward_limit_us,
        feedforward_limit_us);

    s_t4_ball_filtered_feedforward_us = T4Ball_ApproachI32(
        s_t4_ball_filtered_feedforward_us,
        feedforward_target_us,
        feedforward_step_us);

    s_t4_ball_status.chassis_forward_speed_mm_s =
        measured_forward_speed_mm_s;
    s_t4_ball_status.chassis_planned_accel_mm_s2 =
        s_t4_ball_chassis_status.forward_accel_mm_s2;
    s_t4_ball_status.chassis_measured_accel_mm_s2 =
        s_t4_ball_filtered_measured_accel_mm_s2;
    s_t4_ball_status.chassis_turn_command_mm_s =
        s_t4_ball_chassis_status.turn_command_mm_s;
    s_t4_ball_status.acceleration_feedforward_us =
        s_t4_ball_filtered_feedforward_us;
    s_t4_ball_status.launch_boost_active =
        s_t4_ball_launch_active;

    s_t4_ball_status.dynamic_motion =
        (T4Ball_AbsI32(s_t4_ball_chassis_status.forward_accel_mm_s2) >=
             T4_BALL_DYNAMIC_ACCEL_MM_S2) ||
        (T4Ball_AbsI32(s_t4_ball_chassis_status.turn_command_mm_s) >=
             T4_BALL_DYNAMIC_TURN_MM_S) ||
        (T4Ball_AbsI32(s_t4_ball_chassis_status.turn_accel_mm_s2) >=
             T4_BALL_DYNAMIC_TURN_ACCEL_MM_S2) ||
        (T4Ball_AbsI32(measured_forward_speed_mm_s) >=
             T4_BALL_DYNAMIC_SPEED_MM_S);
    return true;
}

static int32_t T4Ball_PositionGain(int32_t abs_predicted_error_px,
                                   int32_t position_deadband_px)
{
    if (abs_predicted_error_px <= position_deadband_px)
    {
        return 0;
    }
    if (abs_predicted_error_px <= T4_BALL_POSITION_NEAR_PX)
    {
        return 2;
    }
    if (abs_predicted_error_px <= T4_BALL_POSITION_MID_PX)
    {
        return 3;
    }
    return 4;
}

static bool T4Ball_InitCommon(
    int32_t target_x,
    int32_t target_mm,
    bool capture_current)
{
    memset(&s_t4_ball_status, 0, sizeof(s_t4_ball_status));
    s_t4_ball_initialized = false;
    s_t4_ball_last_vision_sequence = 0U;
    s_t4_ball_last_vision_timestamp_ms = 0U;
    s_t4_ball_last_chassis_timestamp_ms = 0U;
    s_t4_ball_previous_center_x = 0;
    s_t4_ball_previous_forward_speed_mm_s = 0;
    s_t4_ball_filtered_velocity_px_s = 0;
    s_t4_ball_filtered_measured_accel_mm_s2 = 0;
    s_t4_ball_filtered_feedforward_us = 0;
    s_t4_ball_bias_divider = 0U;
    s_t4_ball_last_error_sign = 0;
    s_t4_ball_cross_brake_frames = 0U;
    s_t4_ball_launch_active = !capture_current;
    s_t4_ball_launch_start_ms = 0U;
    s_t4_ball_capture_count = 0U;
    s_t4_ball_capture_last_x = 0;
    s_t4_ball_capture_sum_x = 0;
    s_t4_ball_capture_sum_mm = 0;
    memset(&s_t4_ball_chassis_status, 0,
           sizeof(s_t4_ball_chassis_status));

    s_t4_ball_status.target_x = target_x;
    s_t4_ball_status.target_mm = target_mm;
    s_t4_ball_status.target_locked = !capture_current;
    s_t4_ball_status.capture_target_pending = capture_current;
    s_t4_ball_status.servo_pulse_us =
        (uint16_t)T4_BALL_SERVO_CENTER_US;
    s_t4_ball_status.within_one_cm = true;
    s_t4_ball_status.mode = capture_current ?
        TASK4_MAIN_BALL_MODE_CAPTURE_TARGET :
        TASK4_MAIN_BALL_MODE_WAIT_VISION;

    if (!BSP_Servo_IsInitialized() && !BSP_Servo_Init())
    {
        return false;
    }
    if (!BSP_Servo_SetPulseUs((uint16_t)T4_BALL_SERVO_CENTER_US) ||
        !BSP_Servo_Enable())
    {
        return false;
    }

    s_t4_ball_initialized = true;
    s_t4_ball_status.initialized = true;
    return true;
}

bool Task4MainBall_Init(void)
{
    return Task4MainBall_InitTarget(T4_BALL_TARGET_CX, 0);
}

bool Task4MainBall_InitTarget(int32_t target_x, int32_t target_mm)
{
    if ((target_mm < T4_BALL_CAPTURE_MIN_MM) ||
        (target_mm > T4_BALL_CAPTURE_MAX_MM))
    {
        return false;
    }
    return T4Ball_InitCommon(target_x, target_mm, false);
}

bool Task4MainBall_InitCaptureCurrent(void)
{
    return T4Ball_InitCommon(T4_BALL_TARGET_CX, 0, true);
}

bool Task4MainBall_IsTargetLocked(void)
{
    return s_t4_ball_initialized && s_t4_ball_status.target_locked;
}

bool Task4MainBall_Update(const VisionStatus_t *vision_status)
{
    int32_t center_x;
    int32_t position_mm;
    int32_t error_px;
    int32_t error_mm;
    int32_t frame_delta_px;
    int32_t raw_velocity_px_s;
    int32_t filtered_velocity_px_s;
    int32_t predicted_error_px;
    int32_t position_control_us;
    int32_t damping_control_us;
    int32_t desired_delta_us;
    int32_t desired_pulse_us;
    int32_t maximum_step_us;
    int32_t abs_error_px;
    int32_t abs_error_mm;
    int32_t abs_predicted_error_px;
    int32_t abs_velocity_px_s;
    int32_t position_gain;
    int32_t position_gain_milli;
    int32_t position_scale_milli;
    int32_t velocity_gain_num;
    int32_t velocity_gain_milli;
    int32_t velocity_scale_milli;
    int32_t raw_velocity_gain_milli;
    int32_t raw_velocity_scale_milli;
    int32_t prediction_horizon_ms;
    int32_t prediction_scale_milli;
    int32_t position_deadband_px;
    int32_t target_schedule_milli;
    int32_t extreme_schedule_milli;
    int32_t deadband_extra_px;
    int32_t fast_error_threshold_px;
    int32_t recover_error_threshold_px;
    int32_t fast_velocity_threshold_px_s;
    int32_t recover_velocity_threshold_px_s;
    int32_t current_error_sign;
    int32_t cross_position_scale_milli;
    int32_t cross_damping_scale_milli;
    int32_t cross_slew_us;
    int32_t toward_scale_milli;
    int32_t away_scale_milli;
    int32_t bias_adapt_divider;
    int32_t bias_step_us;
    uint32_t vision_dt_ms = 0U;
    bool vehicle_motion_updated;
    bool cross_brake_active = false;

    if (!s_t4_ball_initialized || (vision_status == 0))
    {
        return false;
    }

    vehicle_motion_updated = T4Ball_UpdateVehicleMotion();
    target_schedule_milli = T4Ball_TargetScheduleMilli();
    extreme_schedule_milli = T4Ball_ExtremeScheduleMilli();
    deadband_extra_px = T4Ball_ScheduledValue(
        0,
        T4_BALL_OFFCENTER_DEADBAND_EXTRA_PX,
        target_schedule_milli);
    deadband_extra_px = T4Ball_ScheduledValue(
        deadband_extra_px,
        T4_BALL_EXTREME_DEADBAND_EXTRA_PX,
        extreme_schedule_milli);
    position_deadband_px = T4_BALL_POSITION_DEADBAND_PX +
        deadband_extra_px;
    fast_error_threshold_px = T4_BALL_FAST_ERROR_PX +
        T4Ball_MulDivI32(
            T4_BALL_EXTREME_FAST_ERROR_EXTRA_PX,
            extreme_schedule_milli,
            T4_BALL_SCHEDULE_ONE_MILLI);
    recover_error_threshold_px = T4_BALL_RECOVER_ERROR_PX +
        T4Ball_MulDivI32(
            T4_BALL_EXTREME_RECOVER_ERROR_EXTRA_PX,
            extreme_schedule_milli,
            T4_BALL_SCHEDULE_ONE_MILLI);
    fast_velocity_threshold_px_s = T4_BALL_FAST_VELOCITY_PX_S +
        T4Ball_MulDivI32(
            T4_BALL_EXTREME_FAST_VELOCITY_EXTRA_PX_S,
            extreme_schedule_milli,
            T4_BALL_SCHEDULE_ONE_MILLI);
    recover_velocity_threshold_px_s =
        T4_BALL_RECOVER_VELOCITY_PX_S +
        T4Ball_MulDivI32(
            T4_BALL_EXTREME_RECOVER_VELOCITY_EXTRA_PX_S,
            extreme_schedule_milli,
            T4_BALL_SCHEDULE_ONE_MILLI);
    s_t4_ball_status.target_schedule_milli = target_schedule_milli;
    s_t4_ball_status.extreme_schedule_milli = extreme_schedule_milli;
    s_t4_ball_status.position_deadband_px = position_deadband_px;
    s_t4_ball_status.fast_error_threshold_px = fast_error_threshold_px;
    s_t4_ball_status.recover_error_threshold_px =
        recover_error_threshold_px;

    s_t4_ball_status.vision_sequence = vision_status->sequence;
    s_t4_ball_status.vision_valid =
        vision_status->has_frame &&
        vision_status->data_valid &&
        vision_status->frame.found;

    if (!s_t4_ball_status.vision_valid)
    {
        s_t4_ball_status.lost_frames++;
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_VISION_LOST;
        s_t4_ball_previous_center_x = 0;
        s_t4_ball_last_vision_timestamp_ms = 0U;
        s_t4_ball_filtered_velocity_px_s = 0;
        s_t4_ball_last_error_sign = 0;
        s_t4_ball_cross_brake_frames = 0U;
        s_t4_ball_status.cross_brake_frames = 0U;
        s_t4_ball_status.raw_speed_px = 0;
        s_t4_ball_status.filtered_speed_px = 0;
        s_t4_ball_status.raw_velocity_px_s = 0;
        s_t4_ball_status.filtered_velocity_px_s = 0;
        s_t4_ball_status.position_control_us = 0;
        s_t4_ball_status.damping_control_us = 0;
        s_t4_ball_status.control_delta_us =
            s_t4_ball_status.learned_bias_us +
            s_t4_ball_status.acceleration_feedforward_us;
        return T4Ball_WritePulse(
            T4_BALL_SERVO_CENTER_US +
                s_t4_ball_status.control_delta_us,
            T4Ball_ScheduledSlew(
                T4_BALL_SLEW_FAST_US,
                target_schedule_milli,
                false));
    }

    if (s_t4_ball_status.capture_target_pending)
    {
        if (vision_status->sequence == s_t4_ball_last_vision_sequence)
        {
            return true;
        }
        s_t4_ball_last_vision_sequence = vision_status->sequence;
        s_t4_ball_status.control_sequence++;
        center_x = (int32_t)vision_status->frame.center_x;
        position_mm = (int32_t)vision_status->frame.physical_x_mm;
        s_t4_ball_status.center_x = center_x;
        s_t4_ball_status.position_mm = position_mm;
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_CAPTURE_TARGET;
        s_t4_ball_status.lost_frames = 0U;

        if ((position_mm < T4_BALL_CAPTURE_MIN_MM) ||
            (position_mm > T4_BALL_CAPTURE_MAX_MM))
        {
            s_t4_ball_capture_count = 0U;
            s_t4_ball_capture_last_x = center_x;
            s_t4_ball_capture_sum_x = 0;
            s_t4_ball_capture_sum_mm = 0;
            return T4Ball_WritePulse(
                T4_BALL_SERVO_CENTER_US,
                T4_BALL_SLEW_HOLD_US);
        }

        if ((s_t4_ball_capture_count == 0U) ||
            (T4Ball_AbsI32(center_x - s_t4_ball_capture_last_x) <=
             T4_BALL_CAPTURE_MAX_DELTA_PX))
        {
            s_t4_ball_capture_count++;
            s_t4_ball_capture_sum_x += center_x;
            s_t4_ball_capture_sum_mm += position_mm;
        }
        else
        {
            s_t4_ball_capture_count = 1U;
            s_t4_ball_capture_sum_x = center_x;
            s_t4_ball_capture_sum_mm = position_mm;
        }
        s_t4_ball_capture_last_x = center_x;

        if (s_t4_ball_capture_count >=
            T4_BALL_CAPTURE_REQUIRED_FRAMES)
        {
            s_t4_ball_status.target_x =
                s_t4_ball_capture_sum_x /
                (int32_t)s_t4_ball_capture_count;
            s_t4_ball_status.target_mm =
                s_t4_ball_capture_sum_mm /
                (int32_t)s_t4_ball_capture_count;
            s_t4_ball_status.target_locked = true;
            s_t4_ball_status.capture_target_pending = false;
            s_t4_ball_status.error_px = 0;
            s_t4_ball_status.error_mm = 0;
            s_t4_ball_status.max_abs_error_px = 0U;
            s_t4_ball_status.max_abs_error_mm = 0U;
            s_t4_ball_status.one_cm_violation_latched = false;
            s_t4_ball_status.within_one_cm = true;
            s_t4_ball_previous_center_x = center_x;
            s_t4_ball_last_vision_timestamp_ms =
                vision_status->timestamp_ms;
            s_t4_ball_filtered_velocity_px_s = 0;
            s_t4_ball_last_error_sign = 0;
            s_t4_ball_cross_brake_frames = 0U;
            s_t4_ball_status.cross_brake_frames = 0U;
            s_t4_ball_launch_active = true;
            s_t4_ball_launch_start_ms = 0U;
            s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_HOLD;
            (void)BSP_Debug_Printf(
                "BALL_HOLD,TARGET_LOCK=1,CX=%ld,XMM=%ld,N=%u\r\n",
                (long)s_t4_ball_status.target_x,
                (long)s_t4_ball_status.target_mm,
                (unsigned int)s_t4_ball_capture_count);
        }

        return T4Ball_WritePulse(
            T4_BALL_SERVO_CENTER_US,
            T4_BALL_SLEW_HOLD_US);
    }

    if (vision_status->sequence == s_t4_ball_last_vision_sequence)
    {
        if (vehicle_motion_updated)
        {
            desired_delta_us =
                s_t4_ball_status.learned_bias_us +
                s_t4_ball_status.acceleration_feedforward_us +
                s_t4_ball_status.position_control_us +
                s_t4_ball_status.damping_control_us;
            desired_delta_us = T4Ball_ClampI32(
                desired_delta_us,
                T4_BALL_CONTROL_NEG_LIMIT_US,
                T4_BALL_CONTROL_POS_LIMIT_US);
            s_t4_ball_status.control_delta_us = desired_delta_us;
            maximum_step_us = T4Ball_ScheduledSlew(
                s_t4_ball_status.dynamic_motion ?
                    T4_BALL_SLEW_FAST_US : T4_BALL_SLEW_NORMAL_US,
                target_schedule_milli,
                false);
            return T4Ball_WritePulse(
                T4_BALL_SERVO_CENTER_US + desired_delta_us,
                maximum_step_us);
        }
        return true;
    }
    s_t4_ball_last_vision_sequence = vision_status->sequence;
    s_t4_ball_status.control_sequence++;
    s_t4_ball_status.lost_frames = 0U;

    center_x = (int32_t)vision_status->frame.center_x;
    position_mm = (int32_t)vision_status->frame.physical_x_mm;
    error_px = center_x - s_t4_ball_status.target_x;
    error_mm = position_mm - s_t4_ball_status.target_mm;
    frame_delta_px = 0;
    raw_velocity_px_s = 0;

    if ((s_t4_ball_previous_center_x != 0) &&
        (s_t4_ball_last_vision_timestamp_ms != 0U) &&
        (vision_status->timestamp_ms >
         s_t4_ball_last_vision_timestamp_ms))
    {
        vision_dt_ms = vision_status->timestamp_ms -
                       s_t4_ball_last_vision_timestamp_ms;
        if ((vision_dt_ms >= 8U) && (vision_dt_ms <= 120U))
        {
            frame_delta_px = center_x - s_t4_ball_previous_center_x;
            raw_velocity_px_s = (int32_t)(
                ((int64_t)frame_delta_px * 1000LL) /
                (int64_t)vision_dt_ms);
            raw_velocity_px_s = T4Ball_ClampI32(
                raw_velocity_px_s,
                -T4_BALL_RAW_VELOCITY_LIMIT_PX_S,
                T4_BALL_RAW_VELOCITY_LIMIT_PX_S);
        }
    }

    s_t4_ball_previous_center_x = center_x;
    s_t4_ball_last_vision_timestamp_ms = vision_status->timestamp_ms;
    s_t4_ball_filtered_velocity_px_s =
        (s_t4_ball_filtered_velocity_px_s * 3 +
         raw_velocity_px_s) / 4;
    filtered_velocity_px_s = s_t4_ball_filtered_velocity_px_s;

    prediction_scale_milli = T4Ball_ScheduledValue(
        T4_BALL_SCHEDULE_ONE_MILLI,
        T4_BALL_OFFCENTER_PREDICT_SCALE_MILLI,
        target_schedule_milli);
    prediction_scale_milli = T4Ball_ScheduledValue(
        prediction_scale_milli,
        T4_BALL_EXTREME_PREDICT_SCALE_MILLI,
        extreme_schedule_milli);
    prediction_horizon_ms = T4Ball_MulDivI32(
        s_t4_ball_status.dynamic_motion ?
            T4_BALL_PREDICT_DYNAMIC_MS :
            T4_BALL_PREDICT_NORMAL_MS,
        prediction_scale_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
    s_t4_ball_status.prediction_horizon_ms = prediction_horizon_ms;
    predicted_error_px = error_px + (int32_t)(
        ((int64_t)filtered_velocity_px_s *
         (int64_t)prediction_horizon_ms) / 1000LL);
    predicted_error_px = T4Ball_ClampI32(
        predicted_error_px,
        -T4_BALL_PREDICT_LIMIT_PX,
        T4_BALL_PREDICT_LIMIT_PX);

    abs_error_px = T4Ball_AbsI32(error_px);
    abs_error_mm = T4Ball_AbsI32(error_mm);
    abs_predicted_error_px = T4Ball_AbsI32(predicted_error_px);
    abs_velocity_px_s = T4Ball_AbsI32(filtered_velocity_px_s);

    current_error_sign = 0;
    if (error_px > position_deadband_px)
    {
        current_error_sign = 1;
    }
    else if (error_px < -position_deadband_px)
    {
        current_error_sign = -1;
    }
    if ((extreme_schedule_milli > 0) &&
        (current_error_sign != 0))
    {
        if ((s_t4_ball_last_error_sign != 0) &&
            (current_error_sign != s_t4_ball_last_error_sign) &&
            (abs_error_px <= T4_BALL_CROSS_BRAKE_MAX_ERROR_PX))
        {
            s_t4_ball_cross_brake_frames =
                T4_BALL_CROSS_BRAKE_FRAMES;
        }
        s_t4_ball_last_error_sign = (int8_t)current_error_sign;
    }
    else if (extreme_schedule_milli == 0)
    {
        s_t4_ball_last_error_sign = (int8_t)current_error_sign;
        s_t4_ball_cross_brake_frames = 0U;
    }
    if ((s_t4_ball_cross_brake_frames > 0U) &&
        (abs_error_px <= T4_BALL_CROSS_BRAKE_MAX_ERROR_PX) &&
        (extreme_schedule_milli > 0))
    {
        cross_brake_active = true;
    }
    else if (abs_error_px > T4_BALL_CROSS_BRAKE_MAX_ERROR_PX)
    {
        s_t4_ball_cross_brake_frames = 0U;
    }
    s_t4_ball_status.cross_brake_frames =
        s_t4_ball_cross_brake_frames;

    s_t4_ball_status.center_x = center_x;
    s_t4_ball_status.position_mm = position_mm;
    s_t4_ball_status.error_px = error_px;
    s_t4_ball_status.error_mm = error_mm;
    s_t4_ball_status.predicted_error_px = predicted_error_px;
    s_t4_ball_status.raw_speed_px = frame_delta_px;
    s_t4_ball_status.filtered_speed_px =
        (vision_dt_ms > 0U) ?
        (int32_t)(((int64_t)filtered_velocity_px_s *
                   (int64_t)vision_dt_ms) / 1000LL) : 0;
    s_t4_ball_status.raw_velocity_px_s = raw_velocity_px_s;
    s_t4_ball_status.filtered_velocity_px_s = filtered_velocity_px_s;
    s_t4_ball_status.vision_dt_ms = (uint16_t)vision_dt_ms;
    s_t4_ball_status.within_one_cm =
        (abs_error_mm <= T4_BALL_ONE_CM_MM);

    if ((uint32_t)abs_error_px > s_t4_ball_status.max_abs_error_px)
    {
        s_t4_ball_status.max_abs_error_px = (uint32_t)abs_error_px;
    }
    if ((uint32_t)abs_error_mm > s_t4_ball_status.max_abs_error_mm)
    {
        s_t4_ball_status.max_abs_error_mm = (uint32_t)abs_error_mm;
    }
    if (!s_t4_ball_status.within_one_cm)
    {
        s_t4_ball_status.one_cm_violation_latched = true;
    }

    /* Learn only the slow mechanism bias, never a transient vehicle force. */
    if ((T4Ball_AbsI32(
             s_t4_ball_status.chassis_planned_accel_mm_s2) < 60) &&
        (T4Ball_AbsI32(
             s_t4_ball_status.chassis_measured_accel_mm_s2) < 120) &&
        (T4Ball_AbsI32(
             s_t4_ball_status.chassis_turn_command_mm_s) < 50) &&
        (abs_error_px >= T4_BALL_BIAS_ADAPT_MIN_ERROR_PX) &&
        (abs_error_px <= T4_BALL_BIAS_ADAPT_MAX_ERROR_PX) &&
        (abs_velocity_px_s <= T4_BALL_BIAS_ADAPT_MAX_VELOCITY_PX_S) &&
        (T4Ball_AbsI32(raw_velocity_px_s) <=
         T4_BALL_BIAS_ADAPT_MAX_VELOCITY_PX_S))
    {
        bias_adapt_divider = T4_BALL_BIAS_ADAPT_DIVIDER +
            T4Ball_MulDivI32(
                T4_BALL_OFFCENTER_BIAS_DIVIDER_EXTRA,
                target_schedule_milli,
                T4_BALL_SCHEDULE_ONE_MILLI);
        bias_adapt_divider += T4Ball_MulDivI32(
            T4_BALL_EXTREME_BIAS_DIVIDER_EXTRA -
                T4_BALL_OFFCENTER_BIAS_DIVIDER_EXTRA,
            extreme_schedule_milli,
            T4_BALL_SCHEDULE_ONE_MILLI);
        s_t4_ball_bias_divider++;
        if ((int32_t)s_t4_ball_bias_divider >= bias_adapt_divider)
        {
            s_t4_ball_bias_divider = 0U;
            bias_step_us = (abs_error_px >= 15) ? 2 : 1;
            if ((target_schedule_milli >= 500) &&
                (bias_step_us > 1))
            {
                bias_step_us = 1;
            }
            if (error_px > 0)
            {
                s_t4_ball_status.learned_bias_us -= bias_step_us;
            }
            else if (error_px < 0)
            {
                s_t4_ball_status.learned_bias_us += bias_step_us;
            }
            s_t4_ball_status.learned_bias_us = T4Ball_ClampI32(
                s_t4_ball_status.learned_bias_us,
                -T4_BALL_BIAS_LIMIT_US,
                T4_BALL_BIAS_LIMIT_US);
        }
    }
    else
    {
        s_t4_ball_bias_divider = 0U;
    }

    position_gain = T4Ball_PositionGain(
        abs_predicted_error_px,
        position_deadband_px);
    position_scale_milli = T4Ball_ScheduledValue(
        T4_BALL_SCHEDULE_ONE_MILLI,
        T4_BALL_OFFCENTER_POSITION_SCALE_MILLI,
        target_schedule_milli);
    position_scale_milli = T4Ball_ScheduledValue(
        position_scale_milli,
        T4_BALL_EXTREME_POSITION_SCALE_MILLI,
        extreme_schedule_milli);
    position_gain_milli = T4Ball_MulDivI32(
        position_gain * T4_BALL_SCHEDULE_ONE_MILLI,
        position_scale_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
    position_control_us = -T4Ball_MulDivI32(
        predicted_error_px,
        position_gain_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
    s_t4_ball_status.position_gain_milli = position_gain_milli;

    /* Reduce position push while already moving toward target; boost while escaping. */
    if ((error_px != 0) && (filtered_velocity_px_s != 0))
    {
        if (((error_px > 0) && (filtered_velocity_px_s < 0)) ||
            ((error_px < 0) && (filtered_velocity_px_s > 0)))
        {
            toward_scale_milli = T4Ball_ScheduledValue(
                750,
                500,
                extreme_schedule_milli);
            position_control_us = T4Ball_MulDivI32(
                position_control_us,
                toward_scale_milli,
                T4_BALL_SCHEDULE_ONE_MILLI);
        }
        else
        {
            away_scale_milli = T4Ball_ScheduledValue(
                1250,
                1050,
                extreme_schedule_milli);
            position_control_us = T4Ball_MulDivI32(
                position_control_us,
                away_scale_milli,
                T4_BALL_SCHEDULE_ONE_MILLI);
        }
    }

    velocity_gain_num = s_t4_ball_status.dynamic_motion ?
        T4_BALL_VELOCITY_GAIN_DYNAMIC_NUM :
        T4_BALL_VELOCITY_GAIN_NORMAL_NUM;
    velocity_scale_milli = T4Ball_ScheduledValue(
        T4_BALL_SCHEDULE_ONE_MILLI,
        T4_BALL_OFFCENTER_VELOCITY_SCALE_MILLI,
        target_schedule_milli);
    velocity_scale_milli = T4Ball_ScheduledValue(
        velocity_scale_milli,
        T4_BALL_EXTREME_VELOCITY_SCALE_MILLI,
        extreme_schedule_milli);
    velocity_gain_milli = T4Ball_MulDivI32(
        velocity_gain_num * 100,
        velocity_scale_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
    raw_velocity_scale_milli = T4Ball_ScheduledValue(
        T4_BALL_SCHEDULE_ONE_MILLI,
        T4_BALL_OFFCENTER_RAW_VEL_SCALE_MILLI,
        target_schedule_milli);
    raw_velocity_scale_milli = T4Ball_ScheduledValue(
        raw_velocity_scale_milli,
        T4_BALL_EXTREME_RAW_VEL_SCALE_MILLI,
        extreme_schedule_milli);
    raw_velocity_gain_milli = T4Ball_MulDivI32(
        T4_BALL_RAW_VELOCITY_GAIN_NUM * 100,
        raw_velocity_scale_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
    damping_control_us = -T4Ball_MulDivI32(
        filtered_velocity_px_s,
        velocity_gain_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
    damping_control_us -= T4Ball_MulDivI32(
        raw_velocity_px_s,
        raw_velocity_gain_milli,
        T4_BALL_SCHEDULE_ONE_MILLI);
    s_t4_ball_status.velocity_gain_milli = velocity_gain_milli;

    if (cross_brake_active)
    {
        cross_position_scale_milli = T4Ball_ScheduledValue(
            T4_BALL_SCHEDULE_ONE_MILLI,
            T4_BALL_CROSS_POSITION_SCALE_MILLI,
            extreme_schedule_milli);
        cross_damping_scale_milli = T4Ball_ScheduledValue(
            T4_BALL_SCHEDULE_ONE_MILLI,
            T4_BALL_CROSS_DAMPING_SCALE_MILLI,
            extreme_schedule_milli);
        position_control_us = T4Ball_MulDivI32(
            position_control_us,
            cross_position_scale_milli,
            T4_BALL_SCHEDULE_ONE_MILLI);
        damping_control_us = T4Ball_MulDivI32(
            damping_control_us,
            cross_damping_scale_milli,
            T4_BALL_SCHEDULE_ONE_MILLI);
    }

    if ((abs_error_px <= position_deadband_px) &&
        (abs_velocity_px_s <= 25) &&
        (T4Ball_AbsI32(raw_velocity_px_s) <= 35) &&
        (!s_t4_ball_launch_active) &&
        (T4Ball_AbsI32(
             s_t4_ball_status.acceleration_feedforward_us) <= 20))
    {
        position_control_us = 0;
        damping_control_us = 0;
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_HOLD;
        maximum_step_us = T4Ball_ScheduledSlew(
            T4_BALL_SLEW_HOLD_US,
            target_schedule_milli,
            false);
    }
    else if (cross_brake_active)
    {
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_DAMP;
        cross_slew_us = T4Ball_ScheduledSlew(
            T4_BALL_SLEW_NORMAL_US,
            target_schedule_milli,
            false);
        maximum_step_us = T4Ball_ScheduledValue(
            cross_slew_us,
            T4_BALL_CROSS_SLEW_US,
            extreme_schedule_milli);
    }
    else if ((abs_error_px >= recover_error_threshold_px) ||
             (abs_velocity_px_s >= recover_velocity_threshold_px_s) ||
             (T4Ball_AbsI32(raw_velocity_px_s) >=
              recover_velocity_threshold_px_s))
    {
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_RECOVER;
        maximum_step_us = T4Ball_ScheduledSlew(
            T4_BALL_SLEW_RECOVER_US,
            target_schedule_milli,
            true);
    }
    else if ((abs_error_px >= fast_error_threshold_px) ||
             (abs_velocity_px_s >= fast_velocity_threshold_px_s) ||
             s_t4_ball_status.dynamic_motion)
    {
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_DAMP;
        maximum_step_us = T4Ball_ScheduledSlew(
            T4_BALL_SLEW_FAST_US,
            target_schedule_milli,
            false);
    }
    else
    {
        s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_CORRECT;
        maximum_step_us = T4Ball_ScheduledSlew(
            T4_BALL_SLEW_NORMAL_US,
            target_schedule_milli,
            false);
    }

    s_t4_ball_status.position_control_us = position_control_us;
    s_t4_ball_status.damping_control_us = damping_control_us;

    desired_delta_us =
        s_t4_ball_status.learned_bias_us +
        s_t4_ball_status.acceleration_feedforward_us +
        position_control_us +
        damping_control_us;
    desired_delta_us = T4Ball_ClampI32(
        desired_delta_us,
        T4_BALL_CONTROL_NEG_LIMIT_US,
        T4_BALL_CONTROL_POS_LIMIT_US);
    desired_pulse_us = T4_BALL_SERVO_CENTER_US + desired_delta_us;

    s_t4_ball_status.control_delta_us = desired_delta_us;
    if (cross_brake_active && (s_t4_ball_cross_brake_frames > 0U))
    {
        s_t4_ball_cross_brake_frames--;
    }
    s_t4_ball_status.cross_brake_frames =
        s_t4_ball_cross_brake_frames;
    return T4Ball_WritePulse(desired_pulse_us, maximum_step_us);
}

void Task4MainBall_Stop(void)
{
    /*
     * Keep PWM enabled at the calibrated horizontal pulse. This covers task
     * exit, menu reset, and fault-reset paths without leaving the rod tilted.
     */
    (void)Project_ServoHoldHorizontal();
    s_t4_ball_initialized = false;
    s_t4_ball_status.initialized = false;
    s_t4_ball_status.servo_pulse_us = PROJECT_SERVO_HORIZONTAL_US;
    s_t4_ball_status.mode = TASK4_MAIN_BALL_MODE_IDLE;
}

bool Task4MainBall_IsInitialized(void)
{
    return s_t4_ball_initialized;
}

bool Task4MainBall_GetStatus(Task4MainBallStatus_t *status)
{
    if ((status == 0) || !s_t4_ball_initialized)
    {
        return false;
    }
    *status = s_t4_ball_status;
    return true;
}

const char *Task4MainBall_ModeName(Task4MainBallMode_t mode)
{
    switch (mode)
    {
        case TASK4_MAIN_BALL_MODE_IDLE:
            return "IDLE";
        case TASK4_MAIN_BALL_MODE_WAIT_VISION:
            return "WAIT";
        case TASK4_MAIN_BALL_MODE_CAPTURE_TARGET:
            return "CAPTURE";
        case TASK4_MAIN_BALL_MODE_HOLD:
            return "HOLD";
        case TASK4_MAIN_BALL_MODE_DAMP:
            return "DAMP";
        case TASK4_MAIN_BALL_MODE_CORRECT:
            return "CORRECT";
        case TASK4_MAIN_BALL_MODE_RECOVER:
            return "RECOVER";
        case TASK4_MAIN_BALL_MODE_VISION_LOST:
            return "VISION_LOST";
        case TASK4_MAIN_BALL_MODE_SERVO_ERROR:
            return "SERVO_ERROR";
        default:
            return "UNKNOWN";
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  /*
   * Hardware RESET clears TIM1 output. Re-enable it immediately after all
   * CubeMX peripheral initialization so the rod returns to horizontal before
   * the menu or any task starts.
   */
  if (!Project_ServoHoldHorizontal())
  {
    Error_Handler();
  }

#if PROJECT_ENTRY_MODE == PROJECT_ENTRY_MAIN_APP
  if (!MainApp_Init())
  {
    Error_Handler();
  }
#elif PROJECT_ENTRY_MODE == PROJECT_ENTRY_TEST_RUNNER
  if (!TestRunner_Init())
  {
    Error_Handler();
  }
#else
#error "Unsupported PROJECT_ENTRY_MODE"
#endif

  /* MainApp initialization may run a force-stop path; reassert level once. */
  if (!Project_ServoHoldHorizontal())
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if PROJECT_ENTRY_MODE == PROJECT_ENTRY_MAIN_APP
    MainApp_Update();
#else
    TestRunner_Update();
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_DebugUart_TxCpltCallback(huart);
    BSP_LineUart_TxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_LineUart_RxCpltCallback(huart);
    BSP_VisionUart_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_DebugUart_ErrorCallback(huart);
    BSP_LineUart_ErrorCallback(huart);
    BSP_VisionUart_ErrorCallback(huart);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_Oled_TxCpltCallback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_Oled_ErrorCallback(hi2c);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  BSP_LineAdc_ConvCpltCallback(hadc);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  BSP_LineAdc_ErrorCallback(hadc);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
