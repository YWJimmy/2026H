#ifndef LINE_FOLLOW_CONTROL_H
#define LINE_FOLLOW_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "line_follow.h"

typedef enum
{
    LINE_FOLLOW_CONTROL_MODE_IDLE = 0,
    LINE_FOLLOW_CONTROL_MODE_WAITING_LINE,
    LINE_FOLLOW_CONTROL_MODE_NORMAL,
    LINE_FOLLOW_CONTROL_MODE_LOST_SEARCH,
    LINE_FOLLOW_CONTROL_MODE_ALL_BLACK_PASS,
    LINE_FOLLOW_CONTROL_MODE_STOPPED
} LineFollowControlMode_t;

typedef enum
{
    LINE_FOLLOW_CONTROL_STOP_NONE = 0,
    LINE_FOLLOW_CONTROL_STOP_USER,
    LINE_FOLLOW_CONTROL_STOP_INVALID_DATA,
    LINE_FOLLOW_CONTROL_STOP_LOST_TIMEOUT,
    LINE_FOLLOW_CONTROL_STOP_LOST_NO_DIRECTION,
    LINE_FOLLOW_CONTROL_STOP_ALL_BLACK_TIMEOUT,
    LINE_FOLLOW_CONTROL_STOP_COMMAND_ERROR
} LineFollowControlStopReason_t;

typedef struct
{
    bool initialized;
    bool running;
    bool chassis_enabled;
    bool has_normal_direction;

    LineFollowControlMode_t mode;
    LineFollowControlStopReason_t stop_reason;
    LineFollowState_t line_state;

    uint8_t black_mask;
    int16_t error;
    int16_t error_delta;

    int32_t base_speed_mm_s;
    int32_t correction_mm_s;
    int32_t left_target_mm_s;
    int32_t right_target_mm_s;

    int8_t last_search_direction;

    uint32_t line_sequence;
    uint32_t state_elapsed_ms;
    uint32_t timestamp_ms;
} LineFollowControlStatus_t;

/**
 * @brief 初始化底盘和巡线外环。
 *
 * 初始化后底盘保持禁用，不会运动。
 */
bool LineFollowControl_Init(void);

/**
 * @brief 开始一次巡线运行。
 *
 * 会重置PD历史、丢线方向和状态计时，并使能底盘。
 * 开始后在收到第一帧有效巡线结果前保持零速。
 */
bool LineFollowControl_Start(void);

/**
 * @brief 以指定原因停止本次巡线。
 *
 * 停止后目标清零并保持短路刹车；不会立即拉低TB6612 STBY。
 * 再次调用Start()可重新开始。
 */
void LineFollowControl_Stop(LineFollowControlStopReason_t reason);

/**
 * @brief 测试结束时彻底关闭底盘。
 *
 * 先短路刹车，再关闭TB6612 STBY。
 */
void LineFollowControl_Shutdown(void);

/**
 * @brief 提交一帧新的巡线分析结果。
 *
 * 仅在运行状态下产生运动命令；停止状态下只保存诊断信息。
 */
bool LineFollowControl_Submit(const LineFollowResult_t *result);

/**
 * @brief 主循环持续调用，使现有底盘5 ms闭环继续运行。
 */
void LineFollowControl_Process(void);

bool LineFollowControl_IsInitialized(void);
bool LineFollowControl_IsRunning(void);
bool LineFollowControl_GetStatus(LineFollowControlStatus_t *status);

const char *LineFollowControl_ModeName(LineFollowControlMode_t mode);
const char *LineFollowControl_StopReasonName(
    LineFollowControlStopReason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_CONTROL_H */
