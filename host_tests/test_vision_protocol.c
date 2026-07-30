#include "vision_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static bool feed_text(VisionProtocolParser_t *parser,
                      const char *text,
                      VisionBallFrame_t *frame)
{
    bool got_frame = false;

    while (*text != '\0')
    {
        if (VisionProtocol_FeedByte(parser, (uint8_t)*text, frame))
        {
            got_frame = true;
        }
        text++;
    }

    return got_frame;
}

static void test_observed_frame(void)
{
    VisionProtocolParser_t parser;
    VisionBallFrame_t frame;

    VisionProtocol_Init(&parser);
    assert(feed_text(&parser, "SB,1,100,200,140,240,120,220,873\r\n", &frame));
    assert(frame.found);
    assert(frame.x1 == 100U);
    assert(frame.y1 == 200U);
    assert(frame.x2 == 140U);
    assert(frame.y2 == 240U);
    assert(frame.center_x == 120U);
    assert(frame.center_y == 220U);
    assert(frame.score_milli == 873U);
    assert(parser.valid_frame_count == 1U);
    assert(parser.invalid_frame_count == 0U);
}

static void test_no_target_and_predicted_frame(void)
{
    VisionProtocolParser_t parser;
    VisionBallFrame_t frame;

    VisionProtocol_Init(&parser);
    assert(feed_text(&parser, "SB,0,0,0,0,0,0,0,0\n", &frame));
    assert(!frame.found);
    assert(feed_text(&parser, "SB,1,10,20,30,40,20,30,0\n", &frame));
    assert(frame.found);
    assert(frame.score_milli == 0U);
    assert(parser.valid_frame_count == 2U);
}

static void test_fragmented_input_and_noise_recovery(void)
{
    VisionProtocolParser_t parser;
    VisionBallFrame_t frame;

    VisionProtocol_Init(&parser);
    assert(!feed_text(&parser, "SB,1,1,2,", &frame));
    assert(!feed_text(&parser, "noise\n", &frame));
    assert(feed_text(&parser, "SB,1,1,2,3,4,2,3,500\n", &frame));
    assert(parser.invalid_frame_count == 1U);
    assert(parser.valid_frame_count == 1U);
}

static void test_rejects_bad_fields_and_ranges(void)
{
    VisionProtocolParser_t parser;
    VisionBallFrame_t frame;

    VisionProtocol_Init(&parser);
    assert(!feed_text(&parser, "SB,1,1,2,3,4,2,3\n", &frame));
    assert(!feed_text(&parser, "SB,2,1,2,3,4,2,3,500\n", &frame));
    assert(!feed_text(&parser, "SB,1,0,0,1280,20,640,10,500\n", &frame));
    assert(!feed_text(&parser, "SB,1,20,0,10,20,15,10,500\n", &frame));
    assert(!feed_text(&parser, "SB,1,10,20,30,40,21,30,500\n", &frame));
    assert(!feed_text(&parser, "SB,0,0,0,0,0,1,0,0\n", &frame));
    assert(parser.invalid_frame_count == 6U);
}

static void test_overlong_line_recovers(void)
{
    VisionProtocolParser_t parser;
    VisionBallFrame_t frame;
    char oversized[VISION_PROTOCOL_LINE_CAPACITY + 24U];

    memset(oversized, 'X', sizeof(oversized));
    oversized[sizeof(oversized) - 2U] = '\n';
    oversized[sizeof(oversized) - 1U] = '\0';

    VisionProtocol_Init(&parser);
    assert(!feed_text(&parser, oversized, &frame));
    assert(parser.overflow_count == 1U);
    assert(parser.invalid_frame_count == 1U);
    assert(feed_text(&parser, "SB,0,0,0,0,0,0,0,0\n", &frame));
}

int main(void)
{
    test_observed_frame();
    test_no_target_and_predicted_frame();
    test_fragmented_input_and_noise_recovery();
    test_rejects_bad_fields_and_ranges();
    test_overlong_line_recovers();
    puts("vision_protocol: PASS");
    return 0;
}
