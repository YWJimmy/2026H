#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { bool found; uint16_t x1,y1,x2,y2,center_x,center_y,score_milli; int16_t physical_x_mm; } VisionBallFrame_t;
#endif
