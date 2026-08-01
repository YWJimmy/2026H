#ifndef VISION_H
#define VISION_H
#include <stdbool.h>
#include <stdint.h>
#include "vision_protocol.h"
typedef struct {
 VisionBallFrame_t frame; uint32_t sequence,timestamp_ms,valid_frame_count,invalid_frame_count,protocol_overflow_count,received_byte_count,uart_overflow_count,uart_error_count,uart_restart_count,last_uart_error; uint16_t queued_byte_count; bool initialized,has_frame,data_valid;
} VisionStatus_t;
bool Vision_Init(void); void Vision_Update(void); void Vision_Stop(void); bool Vision_GetStatus(VisionStatus_t *status);
#endif
