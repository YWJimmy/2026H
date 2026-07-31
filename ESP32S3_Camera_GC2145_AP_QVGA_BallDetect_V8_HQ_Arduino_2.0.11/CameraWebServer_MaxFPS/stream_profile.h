#pragma once

// Fixed Hiwonder GC2145 profile: QVGA RGB565 + software JPEG.
#define STREAM_FRAME_SIZE FRAMESIZE_QVGA
#define STREAM_PROFILE_NAME "QVGA 320x240 / BALL ROI / HQ V8"

// frame2jpg_cb uses 0..100. Lower quality means less CPU time and less Wi-Fi data.
#define SOFTWARE_JPEG_QUALITY 72

// Persistent output buffer, allocated once per active stream.
#define JPEG_BUFFER_CAPACITY (80 * 1024)

// Preserve internal RAM for Wi-Fi/TCP before choosing an internal JPEG buffer.
#define INTERNAL_RAM_RESERVE (96 * 1024)

// Send large JPEG frames in smaller transport chunks.
#define NETWORK_SEND_CHUNK (24 * 1024)


// Run the relatively expensive ROI classifier only on alternate frames.
// The last valid box is still drawn on every streamed frame.
#define BALL_DETECT_EVERY_N_FRAMES 2

// When tracking is valid, scan only around the previous center most of the
// time; periodically scan the full tube to recover from rapid movement.
#define BALL_FULL_SCAN_EVERY_N_FRAMES 8
#define BALL_TRACK_SEARCH_RADIUS 42

// -----------------------------------------------------------------------------
// Steel-ball ROI for the fixed 320x240 camera view.
// Values were derived from the supplied screenshot. Only the inside of the
// horizontal green tube is scanned; the ROI itself is not drawn on the stream.
// -----------------------------------------------------------------------------
#define BALL_ROI_X0 45
#define BALL_ROI_X1 285
#define BALL_ROI_Y0 111
#define BALL_ROI_Y1 128

// Horizontal half-width of the scoring window (11 columns total).
#define BALL_WINDOW_RADIUS 5

// Pixel classification thresholds for the silver ball against the green tube.
#define BALL_BRIGHT_LUMA_MIN 105
#define BALL_BRIGHT_CHROMA_MAX 55
#define BALL_BRIGHT_GREEN_EXCESS_MAX 22
#define BALL_NEUTRAL_CHROMA_MAX 45
#define BALL_NEUTRAL_GREEN_EXCESS_MAX 18

// Window acceptance thresholds. Lowering these increases sensitivity but also
// increases the chance of false boxes on reflections.
#define BALL_MIN_BRIGHT_PIXELS 12
#define BALL_MIN_LOCAL_CONTRAST 90
#define BALL_SCORE_THRESHOLD 250

// Red box and temporal stabilisation.
#define BALL_BOX_HALF_WIDTH 9
#define BALL_BOX_HALF_HEIGHT 9
#define BALL_BOX_THICKNESS 2
#define BALL_HOLD_MISSED_FRAMES 3
