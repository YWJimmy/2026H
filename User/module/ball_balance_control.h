#ifndef BALL_BALANCE_CONTROL_H
#define BALL_BALANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "vision.h"

typedef enum
{
    BALL_BALANCE_MODE_IDLE = 0,
    BALL_BALANCE_MODE_WAITING_VISION,
    BALL_BALANCE_MODE_NO_TARGET,
    BALL_BALANCE_MODE_HOLD,
    BALL_BALANCE_MODE_PUSH,
    BALL_BALANCE_MODE_BRAKE,
    BALL_BALANCE_MODE_VISION_TIMEOUT,
    BALL_BALANCE_MODE_SERVO_ERROR
} BallBalanceMode_t;

typedef enum
{
    BALL_BALANCE_EVENT_NONE = 0,
    BALL_BALANCE_EVENT_LOST_RETURN_CENTER,
    BALL_BALANCE_EVENT_VISION_TIMEOUT_RETURN_CENTER,
    BALL_BALANCE_EVENT_SERVO_COMMAND_ERROR
} BallBalanceEvent_t;

typedef struct
{
    bool initialized;
    bool servo_enabled;
    bool has_target;
    bool vision_has_frame;
    bool vision_data_valid;
    bool braking;

    BallBalanceMode_t mode;
    BallBalanceEvent_t last_event;

    uint32_t control_sequence;
    uint32_t vision_sequence;
    uint32_t event_sequence;
    uint32_t lost_frames;
    uint32_t stuck_boost_us;
    uint32_t servo_error_count;
    uint32_t timestamp_ms;

    int32_t center_x;
    int32_t target_x;
    int32_t error;
    int32_t speed;
    int32_t delta_us;

    uint16_t servo_pulse_us;
} BallBalanceControlStatus_t;

/*
 * 初始化钢球控制模块和舵机BSP。
 * 模块初始化后舵机输出1700 us，不负责初始化Vision模块。
 */
bool BallBalanceControl_Init(void);

/* Update the requested ball center in the vision X coordinate system. */
bool BallBalanceControl_SetTargetX(int32_t target_x);

/*
 * 提交Vision模块的最新状态。
 * 同一sequence只处理一次；无新帧时仍检查data_valid超时。
 */
bool BallBalanceControl_Update(const VisionStatus_t *vision_status);

/* 舵机回中、关闭PWM并结束控制。 */
void BallBalanceControl_Stop(void);

bool BallBalanceControl_IsInitialized(void);
bool BallBalanceControl_GetStatus(BallBalanceControlStatus_t *status);

const char *BallBalanceControl_ModeName(BallBalanceMode_t mode);
const char *BallBalanceControl_EventName(BallBalanceEvent_t event);

#ifdef __cplusplus
}
#endif

#endif /* BALL_BALANCE_CONTROL_H */
