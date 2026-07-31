#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "freertos/FreeRTOS.h"
#include "stream_profile.h"

// ============================================================
// GC2145 QVGA Ball Detect V8 HQ MJPEG server
// - 320x240 RGB565, single framebuffer in PSRAM
// - Persistent 80 KB JPEG buffer; no per-frame allocation/free
// - Exactly one active stream
// - Start first closes any stale stream; Stop actively shuts it down
// - Alternate-frame local/full ROI steel-ball detection and red overlay
// - JPEG quality 72, larger network chunks, no periodic performance logging
// ============================================================

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_HEADER_FORMAT =
    "\r\n--" PART_BOUNDARY "\r\n"
    "Content-Type: image/jpeg\r\n"
    "Content-Length: %u\r\n\r\n";

static httpd_handle_t camera_httpd = nullptr;
static httpd_handle_t stream_httpd = nullptr;

// Shared by the page server (port 80) and stream server (port 81).
static portMUX_TYPE stream_mux = portMUX_INITIALIZER_UNLOCKED;
static int active_stream_fd = -1;
static uint32_t active_stream_token = 0;

struct jpeg_buffer_t {
  uint8_t *data;
  size_t capacity;
  size_t length;
  bool overflow;
};

struct ball_detection_t {
  bool found;
  int x;
  int y;
  int score;
};

struct ball_tracker_t {
  bool valid;
  int x;
  uint8_t missed_frames;
  uint32_t frame_counter;
};

static ball_tracker_t ball_tracker = {
    .valid = false,
    .x = 0,
    .missed_frames = 0,
    .frame_counter = 0,
};

static inline void unpackRgb565BigEndian(const uint8_t *pixel,
                                         uint8_t &r,
                                         uint8_t &g,
                                         uint8_t &b) {
  // esp32-camera's RGB565 JPEG converter treats camera frames as big-endian:
  // first byte = RRRRRGGG, second byte = GGGBBBBB.
  r = pixel[0] & 0xF8;
  g = static_cast<uint8_t>(((pixel[0] & 0x07) << 5) |
                           ((pixel[1] & 0xE0) >> 3));
  b = static_cast<uint8_t>((pixel[1] & 0x1F) << 3);
}

static inline void setRedPixel(camera_fb_t *fb, int x, int y) {
  if (fb == nullptr || x < 0 || y < 0 ||
      x >= static_cast<int>(fb->width) ||
      y >= static_cast<int>(fb->height)) {
    return;
  }
  const size_t index =
      (static_cast<size_t>(y) * fb->width + static_cast<size_t>(x)) * 2U;
  // RGB565 red = 0xF800, stored big-endian in the camera framebuffer.
  fb->buf[index] = 0xF8;
  fb->buf[index + 1] = 0x00;
}

static void drawRedBox(camera_fb_t *fb, int center_x, int center_y) {
  const int left = center_x - BALL_BOX_HALF_WIDTH;
  const int right = center_x + BALL_BOX_HALF_WIDTH;
  const int top = center_y - BALL_BOX_HALF_HEIGHT;
  const int bottom = center_y + BALL_BOX_HALF_HEIGHT;

  for (int thickness = 0; thickness < BALL_BOX_THICKNESS; ++thickness) {
    const int x0 = left - thickness;
    const int x1 = right + thickness;
    const int y0 = top - thickness;
    const int y1 = bottom + thickness;
    for (int x = x0; x <= x1; ++x) {
      setRedPixel(fb, x, y0);
      setRedPixel(fb, x, y1);
    }
    for (int y = y0; y <= y1; ++y) {
      setRedPixel(fb, x0, y);
      setRedPixel(fb, x1, y);
    }
  }
}

static ball_detection_t detectSteelBall(const camera_fb_t *fb, int scan_x0, int scan_x1) {
  ball_detection_t result = {
      .found = false,
      .x = 0,
      .y = (BALL_ROI_Y0 + BALL_ROI_Y1) / 2,
      .score = 0,
  };

  if (fb == nullptr || fb->format != PIXFORMAT_RGB565 ||
      fb->width <= BALL_ROI_X1 || fb->height <= BALL_ROI_Y1) {
    return result;
  }

  if (scan_x0 < BALL_ROI_X0) scan_x0 = BALL_ROI_X0;
  if (scan_x1 > BALL_ROI_X1) scan_x1 = BALL_ROI_X1;
  if (scan_x1 - scan_x0 + 1 <= BALL_WINDOW_RADIUS * 2) {
    return result;
  }

  constexpr int max_roi_width = BALL_ROI_X1 - BALL_ROI_X0 + 1;
  const int scan_width = scan_x1 - scan_x0 + 1;
  uint8_t bright_count[max_roi_width];
  uint8_t neutral_count[max_roi_width];
  uint8_t min_luma[max_roi_width];
  uint8_t max_luma[max_roi_width];

  for (int column = 0; column < scan_width; ++column) {
    const int x = scan_x0 + column;
    unsigned bright = 0;
    unsigned neutral = 0;
    uint8_t local_min = 255;
    uint8_t local_max = 0;

    for (int y = BALL_ROI_Y0; y <= BALL_ROI_Y1; ++y) {
      const size_t index =
          (static_cast<size_t>(y) * fb->width + static_cast<size_t>(x)) * 2U;
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      unpackRgb565BigEndian(fb->buf + index, r, g, b);

      const uint8_t max_channel = (r > g) ? ((r > b) ? r : b)
                                           : ((g > b) ? g : b);
      const uint8_t min_channel = (r < g) ? ((r < b) ? r : b)
                                           : ((g < b) ? g : b);
      const int chroma = static_cast<int>(max_channel) - min_channel;
      const int rb_max = (r > b) ? r : b;
      const int green_excess = static_cast<int>(g) - rb_max;
      const uint8_t luma = static_cast<uint8_t>(
          (77U * r + 150U * g + 29U * b) >> 8);

      if (luma < local_min) local_min = luma;
      if (luma > local_max) local_max = luma;

      if (luma >= BALL_BRIGHT_LUMA_MIN &&
          chroma <= BALL_BRIGHT_CHROMA_MAX &&
          green_excess <= BALL_BRIGHT_GREEN_EXCESS_MAX) {
        ++bright;
      }
      if (chroma <= BALL_NEUTRAL_CHROMA_MAX &&
          green_excess <= BALL_NEUTRAL_GREEN_EXCESS_MAX) {
        ++neutral;
      }
    }

    bright_count[column] = static_cast<uint8_t>(bright);
    neutral_count[column] = static_cast<uint8_t>(neutral);
    min_luma[column] = local_min;
    max_luma[column] = local_max;
  }

  for (int center = BALL_WINDOW_RADIUS;
       center < scan_width - BALL_WINDOW_RADIUS;
       ++center) {
    unsigned bright_sum = 0;
    unsigned neutral_sum = 0;
    uint8_t window_min = 255;
    uint8_t window_max = 0;

    for (int offset = -BALL_WINDOW_RADIUS;
         offset <= BALL_WINDOW_RADIUS;
         ++offset) {
      const int index = center + offset;
      bright_sum += bright_count[index];
      neutral_sum += neutral_count[index];
      if (min_luma[index] < window_min) window_min = min_luma[index];
      if (max_luma[index] > window_max) window_max = max_luma[index];
    }

    const int contrast = static_cast<int>(window_max) - window_min;
    if (bright_sum < BALL_MIN_BRIGHT_PIXELS ||
        contrast < BALL_MIN_LOCAL_CONTRAST) {
      continue;
    }

    // Silver ball: local neutral highlight plus a strong bright/dark transition.
    const int score = static_cast<int>(bright_sum) * 4 +
                      static_cast<int>(neutral_sum) +
                      (contrast * 3) / 2;
    if (score > result.score) {
      result.score = score;
      result.x = scan_x0 + center;
    }
  }

  result.found = (result.score >= BALL_SCORE_THRESHOLD);
  return result;
}

static void detectTrackAndDrawBall(camera_fb_t *fb) {
  ++ball_tracker.frame_counter;

  // High JPEG quality dominates CPU time. Save detector work by classifying
  // alternate frames while keeping the previous red box visible every frame.
  const bool should_detect =
      !ball_tracker.valid ||
      (ball_tracker.frame_counter % BALL_DETECT_EVERY_N_FRAMES == 0);

  if (should_detect) {
    int scan_x0 = BALL_ROI_X0;
    int scan_x1 = BALL_ROI_X1;

    // Usually search near the last center. A periodic full scan recovers when
    // the ball moves quickly or tracking was briefly wrong.
    const bool full_scan =
        !ball_tracker.valid ||
        (ball_tracker.frame_counter % BALL_FULL_SCAN_EVERY_N_FRAMES == 0);
    if (!full_scan) {
      scan_x0 = ball_tracker.x - BALL_TRACK_SEARCH_RADIUS;
      scan_x1 = ball_tracker.x + BALL_TRACK_SEARCH_RADIUS;
    }

    const ball_detection_t detection = detectSteelBall(fb, scan_x0, scan_x1);

    if (detection.found) {
      if (ball_tracker.valid) {
        const int delta = detection.x - ball_tracker.x;
        const int absolute_delta = (delta < 0) ? -delta : delta;
        if (absolute_delta <= 30) {
          // Mild horizontal smoothing without adding a visible tracking delay.
          ball_tracker.x = (ball_tracker.x * 2 + detection.x + 1) / 3;
        } else {
          ball_tracker.x = detection.x;
        }
      } else {
        ball_tracker.x = detection.x;
      }
      ball_tracker.valid = true;
      ball_tracker.missed_frames = 0;
    } else if (ball_tracker.valid) {
      if (ball_tracker.missed_frames < BALL_HOLD_MISSED_FRAMES) {
        ++ball_tracker.missed_frames;
      } else {
        ball_tracker.valid = false;
      }
    }
  }

  if (ball_tracker.valid) {
    drawRedBox(fb,
               ball_tracker.x,
               (BALL_ROI_Y0 + BALL_ROI_Y1) / 2);
  }
}

static size_t jpeg_buffer_callback(void *arg,
                                   size_t index,
                                   const void *data,
                                   size_t len) {
  jpeg_buffer_t *buffer = static_cast<jpeg_buffer_t *>(arg);
  if (index == 0) {
    buffer->length = 0;
    buffer->overflow = false;
  }
  if (data == nullptr || len == 0) {
    return 0;
  }
  if (index + len > buffer->capacity) {
    buffer->overflow = true;
    return 0;
  }

  memcpy(buffer->data + index, data, len);
  const size_t new_length = index + len;
  if (new_length > buffer->length) {
    buffer->length = new_length;
  }
  return len;
}

static void stopActiveStream() {
  int fd_to_stop = -1;
  portENTER_CRITICAL(&stream_mux);
  fd_to_stop = active_stream_fd;
  active_stream_fd = -1;
  ++active_stream_token;
  portEXIT_CRITICAL(&stream_mux);

  if (fd_to_stop >= 0) {
    shutdown(fd_to_stop, SHUT_RDWR);
  }
}

static uint32_t claimStream(int new_fd) {
  int old_fd = -1;
  uint32_t token = 0;

  portENTER_CRITICAL(&stream_mux);
  old_fd = active_stream_fd;
  active_stream_fd = new_fd;
  token = ++active_stream_token;
  portEXIT_CRITICAL(&stream_mux);

  if (old_fd >= 0 && old_fd != new_fd) {
    shutdown(old_fd, SHUT_RDWR);
  }
  return token;
}

static bool streamStillActive(int fd, uint32_t token) {
  bool active = false;
  portENTER_CRITICAL(&stream_mux);
  active = (active_stream_fd == fd && active_stream_token == token);
  portEXIT_CRITICAL(&stream_mux);
  return active;
}

static void releaseStream(int fd, uint32_t token) {
  portENTER_CRITICAL(&stream_mux);
  if (active_stream_fd == fd && active_stream_token == token) {
    active_stream_fd = -1;
  }
  portEXIT_CRITICAL(&stream_mux);
}

static uint8_t *allocateJpegBuffer() {
  const size_t largest_internal =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  // Use fast internal RAM only when it leaves enough contiguous headroom for
  // Wi-Fi/TCP/HTTP. Otherwise use PSRAM rather than starving the network stack.
  if (largest_internal >= JPEG_BUFFER_CAPACITY + INTERNAL_RAM_RESERVE) {
    uint8_t *ptr = static_cast<uint8_t *>(
        heap_caps_malloc(JPEG_BUFFER_CAPACITY,
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (ptr != nullptr) {
      return ptr;
    }
  }

  return static_cast<uint8_t *>(
      heap_caps_malloc(JPEG_BUFFER_CAPACITY,
                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

static const char VIEWER_HTML[] = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ESP32-S3 Steel Ball V8 HQ</title>
<style>
*{box-sizing:border-box}
html,body{margin:0;min-height:100%;background:#000;color:#eee;font-family:Arial,sans-serif}
main{min-height:100vh;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:12px;padding:12px}
#video{display:block;width:min(100vw,960px);max-height:calc(100vh - 82px);aspect-ratio:4/3;object-fit:contain;background:#080808;border:1px solid #333}
.controls{display:flex;gap:12px}
button{min-width:110px;padding:11px 22px;border:0;border-radius:7px;font-size:17px;font-weight:600}
#start{background:#25a244;color:white}
#stop{background:#d64045;color:white}
button:disabled{opacity:.45}
#state{font-size:14px;color:#bbb;min-height:18px}
</style>
</head>
<body>
<main>
  <img id="video" alt="点击 Start 开始图传">
  <div class="controls">
    <button id="start" type="button">Start</button>
    <button id="stop" type="button" disabled>Stop</button>
  </div>
  <div id="state">已停止</div>
</main>
<script>
const video=document.getElementById('video');
const startButton=document.getElementById('start');
const stopButton=document.getElementById('stop');
const state=document.getElementById('state');
const blank='data:image/gif;base64,R0lGODlhAQABAAD/ACwAAAAAAQABAAACADs=';
let running=false;
let actionId=0;
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));

async function closeOldStream(){
  video.src=blank;
  try{await fetch('/stop?t='+Date.now(),{cache:'no-store'});}catch(e){}
}

async function startStream(){
  const myAction=++actionId;
  startButton.disabled=true;
  stopButton.disabled=false;
  state.textContent='正在连接……';

  // Explicitly release a stale port-81 connection before opening a new one.
  await closeOldStream();
  await wait(180);
  if(myAction!==actionId)return;

  running=true;
  video.src='http://'+location.hostname+':81/stream?t='+Date.now();
}

async function stopStream(){
  ++actionId;
  running=false;
  startButton.disabled=false;
  stopButton.disabled=true;
  state.textContent='已停止';
  await closeOldStream();
}

video.onload=()=>{
  if(running)state.textContent='图传中';
};
video.onerror=()=>{
  if(!running)return;
  running=false;
  startButton.disabled=false;
  stopButton.disabled=true;
  state.textContent='连接已中断，请点击 Start';
  closeOldStream();
};
startButton.addEventListener('click',startStream);
stopButton.addEventListener('click',stopStream);
closeOldStream();
</script>
</body>
</html>
)HTML";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, VIEWER_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stop_handler(httpd_req_t *req) {
  stopActiveStream();
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, "OK");
}

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control",
                     "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");

  const int socket_fd = httpd_req_to_sockfd(req);
  int tcp_no_delay = 1;
  int keep_alive = 1;
  setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY,
             &tcp_no_delay, sizeof(tcp_no_delay));
  setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE,
             &keep_alive, sizeof(keep_alive));

  const uint32_t stream_token = claimStream(socket_fd);
  uint8_t *jpeg_memory = allocateJpegBuffer();
  if (jpeg_memory == nullptr) {
    releaseStream(socket_fd, stream_token);
    httpd_resp_send_500(req);
    return ESP_ERR_NO_MEM;
  }

  jpeg_buffer_t jpeg = {
      .data = jpeg_memory,
      .capacity = JPEG_BUFFER_CAPACITY,
      .length = 0,
      .overflow = false,
  };

  char frame_header[128];
  unsigned consecutive_capture_failures = 0;

  while (streamStillActive(socket_fd, stream_token)) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
      if (++consecutive_capture_failures >= 3) {
        res = ESP_FAIL;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    consecutive_capture_failures = 0;

    // Detect only inside the fixed green-tube ROI and draw the red box into
    // the RGB565 framebuffer before JPEG encoding, so the phone sees it.
    detectTrackAndDrawBall(fb);

    jpeg.length = 0;
    jpeg.overflow = false;
    const bool encoded = frame2jpg_cb(fb,
                                      SOFTWARE_JPEG_QUALITY,
                                      jpeg_buffer_callback,
                                      &jpeg);
    esp_camera_fb_return(fb);

    // Skip one bad frame instead of tearing down the entire browser stream.
    if (!encoded || jpeg.overflow || jpeg.length == 0) {
      taskYIELD();
      continue;
    }

    const int header_len = snprintf(frame_header,
                                    sizeof(frame_header),
                                    STREAM_HEADER_FORMAT,
                                    static_cast<unsigned>(jpeg.length));
    if (header_len <= 0 || header_len >= static_cast<int>(sizeof(frame_header))) {
      res = ESP_FAIL;
      break;
    }

    res = httpd_resp_send_chunk(req,
                                frame_header,
                                static_cast<size_t>(header_len));
    if (res != ESP_OK) {
      break;
    }

    size_t offset = 0;
    while (offset < jpeg.length && streamStillActive(socket_fd, stream_token)) {
      size_t send_len = jpeg.length - offset;
      if (send_len > NETWORK_SEND_CHUNK) {
        send_len = NETWORK_SEND_CHUNK;
      }
      res = httpd_resp_send_chunk(
          req,
          reinterpret_cast<const char *>(jpeg.data + offset),
          send_len);
      if (res != ESP_OK) {
        break;
      }
      offset += send_len;
    }
    if (res != ESP_OK) {
      break;
    }
    taskYIELD();
  }

  free(jpeg_memory);
  releaseStream(socket_fd, stream_token);
  return res;
}

void startCameraServer() {
  httpd_config_t page_config = HTTPD_DEFAULT_CONFIG();
  page_config.max_uri_handlers = 5;
  page_config.lru_purge_enable = true;
  page_config.stack_size = 4096;
  page_config.recv_wait_timeout = 10;
  page_config.send_wait_timeout = 10;

  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = nullptr,
  };
  httpd_uri_t stop_uri = {
      .uri = "/stop",
      .method = HTTP_GET,
      .handler = stop_handler,
      .user_ctx = nullptr,
  };

  if (httpd_start(&camera_httpd, &page_config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &stop_uri);
  }

  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port = page_config.ctrl_port + 1;
  stream_config.max_uri_handlers = 2;
  stream_config.lru_purge_enable = true;
  stream_config.stack_size = 8192;
  stream_config.recv_wait_timeout = 10;
  stream_config.send_wait_timeout = 10;

  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = nullptr,
  };

  if (httpd_start(&stream_httpd, &stream_config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
