#include "line_follow.h"

#include <string.h>

static const int16_t s_weights[LINE_SENSOR_COUNT] =
{
    -7000, -5000, -3000, -1000,
     1000,  3000,  5000,  7000
};

static LineFollowResult_t s_result;
static bool s_has_result = false;
static int16_t s_last_normal_position = 0;

static uint8_t LineFollow_CountBits(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1U;
    }

    return count;
}

void LineFollow_Init(void)
{
    memset(&s_result, 0, sizeof(s_result));
    s_result.state = LINE_FOLLOW_STATE_INVALID;
    s_has_result = false;
    s_last_normal_position = 0;
}

bool LineFollow_Update(const LineSensorFrame_t *frame)
{
    LineFollowResult_t next;
    int32_t weighted_sum = 0;
    uint32_t strength_sum = 0U;
    uint8_t i;

    if (frame == NULL)
    {
        return false;
    }

    memset(&next, 0, sizeof(next));
    next.sequence = frame->sequence;
    next.black_mask = frame->black_mask;
    next.black_count = LineFollow_CountBits(frame->black_mask);

    if (frame->valid_mask != 0xFFU)
    {
        next.state = LINE_FOLLOW_STATE_INVALID;
        s_result = next;
        s_has_result = true;
        return true;
    }

    for (i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        strength_sum += frame->strength[i];
        weighted_sum += (int32_t)s_weights[i] *
                        (int32_t)frame->strength[i];
    }

    if (strength_sum > 8000U)
    {
        strength_sum = 8000U;
    }
    next.strength_sum = (uint16_t)strength_sum;

    if (frame->black_mask == 0xFFU)
    {
        next.state = LINE_FOLLOW_STATE_ALL_BLACK;
        next.position = 0;
        next.error = 0;
    }
    else if ((next.black_count == 0U) || (strength_sum == 0U))
    {
        next.state = LINE_FOLLOW_STATE_LOST;
        next.position = s_last_normal_position;
        next.error = s_last_normal_position;
    }
    else
    {
        next.state = LINE_FOLLOW_STATE_NORMAL;
        next.position = (int16_t)(weighted_sum / (int32_t)strength_sum);
        next.error = next.position;
        s_last_normal_position = next.position;
    }

    s_result = next;
    s_has_result = true;
    return true;
}

bool LineFollow_GetResult(LineFollowResult_t *result)
{
    if ((result == NULL) || !s_has_result)
    {
        return false;
    }

    *result = s_result;
    return true;
}

const char *LineFollow_StateName(LineFollowState_t state)
{
    switch (state)
    {
        case LINE_FOLLOW_STATE_NORMAL:
            return "NORMAL";
        case LINE_FOLLOW_STATE_LOST:
            return "LOST";
        case LINE_FOLLOW_STATE_ALL_BLACK:
            return "ALL_BLACK";
        case LINE_FOLLOW_STATE_INVALID:
        default:
            return "INVALID";
    }
}
