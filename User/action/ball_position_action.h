#ifndef BALL_POSITION_ACTION_H
#define BALL_POSITION_ACTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ball_balance_control.h"

typedef enum
{
    BALL_POSITION_ACTION_STATE_IDLE = 0,
    BALL_POSITION_ACTION_STATE_ACQUIRING,
    BALL_POSITION_ACTION_STATE_MOVING,
    BALL_POSITION_ACTION_STATE_HOLDING,
    BALL_POSITION_ACTION_STATE_FAULT
} BallPositionActionState_t;

typedef enum
{
    BALL_POSITION_ACTION_RESULT_RUNNING = 0,
    BALL_POSITION_ACTION_RESULT_REACHED,
    BALL_POSITION_ACTION_RESULT_FAULT
} BallPositionActionResult_t;

typedef struct
{
    int32_t target_x;
    int32_t tolerance_px;
    int32_t settle_speed_px;
    uint8_t stable_frames;
    uint32_t vision_timeout_ms;
    uint32_t move_timeout_ms;
    bool capture_current_target;
} BallPositionActionCommand_t;

typedef struct
{
    bool initialized;
    bool active;
    bool target_locked;
    bool reached;
    bool vision_data_valid;
    bool vision_found;

    BallPositionActionState_t state;
    BallBalanceMode_t control_mode;

    uint32_t start_timestamp_ms;
    uint32_t target_timestamp_ms;
    uint32_t last_found_timestamp_ms;
    uint32_t vision_sequence;
    uint32_t stable_frame_count;
    uint32_t fault_detail;

    int32_t center_x;
    int32_t target_x;
    int32_t error_px;
    int32_t raw_speed_px;
    int32_t filtered_speed_px;
    uint16_t servo_pulse_us;
} BallPositionActionStatus_t;

bool BallPositionAction_Init(void);

void BallPositionAction_DefaultCommand(
    BallPositionActionCommand_t *command,
    int32_t target_x);

bool BallPositionAction_Start(
    const BallPositionActionCommand_t *command,
    uint32_t now_ms);

bool BallPositionAction_Retarget(
    const BallPositionActionCommand_t *command,
    uint32_t now_ms);

BallPositionActionResult_t BallPositionAction_Update(
    uint32_t now_ms);

void BallPositionAction_Stop(void);
void BallPositionAction_ForceSafeStop(void);

bool BallPositionAction_IsActive(void);
bool BallPositionAction_GetStatus(
    BallPositionActionStatus_t *status);
uint32_t BallPositionAction_GetFaultDetail(void);
const char *BallPositionAction_StateName(
    BallPositionActionState_t state);

#ifdef __cplusplus
}
#endif

#endif /* BALL_POSITION_ACTION_H */
