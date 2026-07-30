#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "line_sensor.h"

typedef enum
{
    LINE_FOLLOW_STATE_INVALID = 0,
    LINE_FOLLOW_STATE_NORMAL,
    LINE_FOLLOW_STATE_LOST,
    LINE_FOLLOW_STATE_ALL_BLACK
} LineFollowState_t;

typedef struct
{
    LineFollowState_t state;
    int16_t position;
    int16_t error;
    uint16_t strength_sum;
    uint8_t black_count;
    uint8_t black_mask;
    uint32_t sequence;
} LineFollowResult_t;

void LineFollow_Init(void);
bool LineFollow_Update(const LineSensorFrame_t *frame);
bool LineFollow_GetResult(LineFollowResult_t *result);
const char *LineFollow_StateName(LineFollowState_t state);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_H */
