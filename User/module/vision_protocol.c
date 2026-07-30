#include "vision_protocol.h"

#include <stddef.h>
#include <string.h>

#define VISION_PROTOCOL_VALUE_COUNT      8U

static bool VisionProtocol_ParseUnsigned(const char *line,
                                         uint16_t length,
                                         uint16_t *index,
                                         uint32_t *value)
{
    uint32_t parsed = 0U;
    bool has_digit = false;

    while ((*index < length) &&
           (line[*index] >= '0') &&
           (line[*index] <= '9'))
    {
        uint32_t digit = (uint32_t)(line[*index] - '0');

        if (parsed > 100000U)
        {
            return false;
        }
        parsed = parsed * 10U + digit;
        (*index)++;
        has_digit = true;
    }

    if (!has_digit)
    {
        return false;
    }

    *value = parsed;
    return true;
}

static bool VisionProtocol_ParseLine(const char *line,
                                     uint16_t length,
                                     VisionBallFrame_t *frame)
{
    uint32_t values[VISION_PROTOCOL_VALUE_COUNT];
    uint16_t index = 3U;
    uint8_t field;

    if ((line == NULL) || (frame == NULL) || (length < 4U) ||
        (line[0] != 'S') || (line[1] != 'B') || (line[2] != ','))
    {
        return false;
    }

    for (field = 0U; field < VISION_PROTOCOL_VALUE_COUNT; field++)
    {
        if (!VisionProtocol_ParseUnsigned(line, length, &index,
                                          &values[field]))
        {
            return false;
        }

        if (field + 1U < VISION_PROTOCOL_VALUE_COUNT)
        {
            if ((index >= length) || (line[index] != ','))
            {
                return false;
            }
            index++;
        }
    }

    if (index != length)
    {
        return false;
    }

    if (values[0] > 1U)
    {
        return false;
    }

    if (values[0] == 0U)
    {
        for (field = 1U; field < VISION_PROTOCOL_VALUE_COUNT; field++)
        {
            if (values[field] != 0U)
            {
                return false;
            }
        }
    }
    else
    {
        if ((values[1] >= VISION_PROTOCOL_FRAME_WIDTH) ||
            (values[3] >= VISION_PROTOCOL_FRAME_WIDTH) ||
            (values[5] >= VISION_PROTOCOL_FRAME_WIDTH) ||
            (values[2] >= VISION_PROTOCOL_FRAME_HEIGHT) ||
            (values[4] >= VISION_PROTOCOL_FRAME_HEIGHT) ||
            (values[6] >= VISION_PROTOCOL_FRAME_HEIGHT) ||
            (values[7] > VISION_PROTOCOL_SCORE_MAX) ||
            (values[1] > values[3]) ||
            (values[2] > values[4]) ||
            (values[5] != ((values[1] + values[3]) / 2U)) ||
            (values[6] != ((values[2] + values[4]) / 2U)))
        {
            return false;
        }
    }

    frame->found = values[0] != 0U;
    frame->x1 = (uint16_t)values[1];
    frame->y1 = (uint16_t)values[2];
    frame->x2 = (uint16_t)values[3];
    frame->y2 = (uint16_t)values[4];
    frame->center_x = (uint16_t)values[5];
    frame->center_y = (uint16_t)values[6];
    frame->score_milli = (uint16_t)values[7];
    return true;
}

void VisionProtocol_Init(VisionProtocolParser_t *parser)
{
    if (parser != NULL)
    {
        memset(parser, 0, sizeof(*parser));
    }
}

bool VisionProtocol_FeedByte(VisionProtocolParser_t *parser,
                             uint8_t byte,
                             VisionBallFrame_t *frame)
{
    bool valid;

    if ((parser == NULL) || (frame == NULL))
    {
        return false;
    }

    if (byte == (uint8_t)'\r')
    {
        return false;
    }

    if (byte == (uint8_t)'\n')
    {
        if (parser->discarding)
        {
            parser->discarding = false;
            parser->line_length = 0U;
            return false;
        }

        if (parser->line_length == 0U)
        {
            return false;
        }

        valid = VisionProtocol_ParseLine(parser->line,
                                         parser->line_length,
                                         frame);
        parser->line_length = 0U;
        if (valid)
        {
            parser->valid_frame_count++;
            return true;
        }

        parser->invalid_frame_count++;
        return false;
    }

    if (parser->discarding)
    {
        return false;
    }

    if (parser->line_length >= VISION_PROTOCOL_LINE_CAPACITY)
    {
        parser->discarding = true;
        parser->line_length = 0U;
        parser->overflow_count++;
        parser->invalid_frame_count++;
        return false;
    }

    parser->line[parser->line_length] = (char)byte;
    parser->line_length++;
    return false;
}
