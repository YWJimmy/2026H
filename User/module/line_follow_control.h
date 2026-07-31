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

bool LineFollowControl_Init(void);
bool LineFollowControl_Start(void);
void LineFollowControl_Stop(LineFollowControlStopReason_t reason);
void LineFollowControl_Shutdown(void);
bool LineFollowControl_Submit(const LineFollowResult_t *result);
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
