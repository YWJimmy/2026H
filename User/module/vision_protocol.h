#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define VISION_PROTOCOL_LINE_CAPACITY    96U
#define VISION_PROTOCOL_FRAME_WIDTH      1280U
#define VISION_PROTOCOL_FRAME_HEIGHT     960U
#define VISION_PROTOCOL_SCORE_MAX        1000U

/*
 * 固定相机与摆杆位置下的实测横坐标标定点。
 * 物理坐标以 O 点为 0，图像右侧为正，单位为 mm。
 */
#define VISION_CAL_CX_NEG_120_MM          153U
#define VISION_CAL_CX_NEG_50_MM           447U
#define VISION_CAL_CX_ZERO_MM             653U
#define VISION_CAL_CX_POS_50_MM           869U
#define VISION_CAL_CX_POS_120_MM          1160U
#define VISION_PHYSICAL_X_MIN_MM          (-120)
#define VISION_PHYSICAL_X_MAX_MM          120

typedef struct
{
    bool found;
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    uint16_t center_x;
    uint16_t center_y;
    uint16_t score_milli;
    int16_t physical_x_mm;
} VisionBallFrame_t;

typedef struct
{
    char line[VISION_PROTOCOL_LINE_CAPACITY];
    uint16_t line_length;
    bool discarding;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t overflow_count;
} VisionProtocolParser_t;

void VisionProtocol_Init(VisionProtocolParser_t *parser);

/*
 * 使用五点分段线性标定把像素 cx 换算为实际横坐标（mm）。
 * 标定范围外饱和到 -120 mm 或 +120 mm。
 */
int16_t VisionProtocol_CenterXToPhysicalMm(uint16_t center_x);

/*
 * 逐字节输入以 CRLF 或 LF 结束的 SB 帧。
 * 仅在完成且通过全部字段校验时返回 true 并写入 frame。
 */
bool VisionProtocol_FeedByte(VisionProtocolParser_t *parser,
                             uint8_t byte,
                             VisionBallFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* VISION_PROTOCOL_H */
