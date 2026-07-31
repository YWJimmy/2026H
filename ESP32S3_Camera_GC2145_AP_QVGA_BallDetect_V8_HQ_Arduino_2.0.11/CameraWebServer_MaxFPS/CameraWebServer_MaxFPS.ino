#include "esp_camera.h"
#include <WiFi.h>
#include "esp_wifi.h"

// ============================================================
// Hiwonder ESP32-S3-Cam / GC2145
// QVGA Ball Detect V8 HQ for Arduino-ESP32 2.0.11
// Priority: JPEG quality 72, fixed 320x240, smoother stream, lightweight steel-ball ROI detection.
// ============================================================

//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
#define CAMERA_MODEL_ESP32S3_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_M5STACK_ESP32CAM
//#define CAMERA_MODEL_M5STACK_UNITCAM
//#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_TTGO_T_JOURNAL
//#define CAMERA_MODEL_XIAO_ESP32S3
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3
#include "camera_pins.h"

static constexpr const char *AP_SSID = "HW_ESP32S3CAM";
static constexpr uint8_t AP_CHANNEL = 6;
static constexpr uint8_t AP_MAX_CLIENTS = 1;

#include "stream_profile.h"

void startCameraServer();

static void fillCameraConfig(camera_config_t &config) {
  config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // GC2145 on the Hiwonder module is stable at the original 15 MHz XCLK.
  config.xclk_freq_hz = 15000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = STREAM_FRAME_SIZE;
  config.jpeg_quality = SOFTWARE_JPEG_QUALITY;  // RGB565 capture ignores this.

  // V8: keep the 153600-byte QVGA raw frame out of internal RAM.
  // Internal RAM is reserved for Wi-Fi, TCP, HTTP and the JPEG working buffer.
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
}

static bool startNetwork(IPAddress &camera_ip) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(150);
  WiFi.mode(WIFI_AP);
  delay(150);

  const IPAddress ap_ip(192, 168, 5, 1);
  const IPAddress ap_gateway(192, 168, 5, 1);
  const IPAddress ap_subnet(255, 255, 255, 0);

  if (!WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet)) {
    Serial.println("ERROR: softAPConfig failed");
    return false;
  }
  if (!WiFi.softAP(AP_SSID, NULL, AP_CHANNEL, 0, AP_MAX_CLIENTS)) {
    Serial.println("ERROR: softAP start failed");
    return false;
  }

  camera_ip = WiFi.softAPIP();
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.printf("AP READY: %s\n", AP_SSID);
  Serial.printf("AP IP: %s\n", camera_ip.toString().c_str());
  return true;
}

static bool startCamera(camera_config_t &active_config) {
  fillCameraConfig(active_config);
  Serial.printf("Camera: GC2145 RGB565, 15MHz, FB=1, PSRAM, %s\n",
                STREAM_PROFILE_NAME);

  const esp_err_t err = esp_camera_init(&active_config);
  if (err != ESP_OK) {
    Serial.printf("FATAL: camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

static void configureSensorAndWarmUp(const camera_config_t &active_config) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    sensor->set_framesize(sensor, active_config.frame_size);
    sensor->set_whitebal(sensor, 1);
    sensor->set_awb_gain(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);

    // Sensor-side detail enhancement costs essentially no ESP32 CPU per frame.
    // These controls are supported by the GC2145 driver used by the Hiwonder UI.
    sensor->set_bpc(sensor, 1);
    sensor->set_wpc(sensor, 1);
    sensor->set_raw_gma(sensor, 1);
    sensor->set_lenc(sensor, 1);
    sensor->set_sharpness(sensor, 1);
    sensor->set_contrast(sensor, 1);
    sensor->set_saturation(sensor, 0);
  }

  // Let GC2145 auto exposure/white balance settle before Start is pressed.
  for (int i = 0; i < 6; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb != nullptr) {
      esp_camera_fb_return(fb);
    }
    delay(45);
  }
}

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println();
  Serial.println("ESP32-S3 GC2145 QVGA Ball Detect V8 HQ (Arduino 2.0.11)");
  Serial.printf("PSRAM detected: %s\n", psramFound() ? "YES" : "NO");

  IPAddress camera_ip;
  if (!startNetwork(camera_ip)) {
    Serial.println("FATAL: network initialization failed");
    return;
  }

  if (!psramFound()) {
    Serial.println("FATAL: PSRAM is required for V8 QVGA mode");
    return;
  }

  camera_config_t active_config = {};
  if (!startCamera(active_config)) {
    Serial.println("Hotspot is running, but camera/server was not started.");
    return;
  }

  configureSensorAndWarmUp(active_config);
  startCameraServer();

  Serial.printf("Camera READY: RGB565, %s, XCLK=15MHz, FB=1, PSRAM\n",
                STREAM_PROFILE_NAME);
  Serial.print("Viewer: http://");
  Serial.println(camera_ip);
}

void loop() {
  delay(1000);
}
