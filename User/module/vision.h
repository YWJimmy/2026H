#ifndef VISION_H
#define VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "vision_protocol.h"

#define VISION_DATA_VALID_MS    100U

typedef struct
{
    VisionBallFrame_t frame;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t protocol_overflow_count;
    uint32_t received_byte_count;
    uint32_t uart_overflow_count;
    uint32_t uart_error_count;
    uint32_t uart_restart_count;
    uint32_t last_uart_error;
    uint16_t queued_byte_count;
    bool initialized;
    bool has_frame;
    bool data_valid;
} VisionStatus_t;

bool Vision_Init(void);
void Vision_Update(void);
void Vision_Stop(void);
bool Vision_GetStatus(VisionStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* VISION_H */
