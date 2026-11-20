#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ============================================================================
// CONFIG: UPDATE WITH YOUR 2.4 GHz WI-FI CREDENTIALS
// ============================================================================
const char* AP_SSID     = "ESP32-CAM-Test";
const char* AP_PASSWORD = "password123"; // Must be at least 8 characters

// AI-Thinker ESP32-CAM Pin Assignments
constexpr int PWDN_GPIO_NUM  = 32;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM  = 0;
constexpr int SIOD_GPIO_NUM  = 26;
constexpr int SIOC_GPIO_NUM  = 27;
constexpr int Y9_GPIO_NUM    = 35;
constexpr int Y8_GPIO_NUM    = 34;
constexpr int Y7_GPIO_NUM    = 39;
constexpr int Y6_GPIO_NUM    = 36;
constexpr int Y5_GPIO_NUM    = 21;
constexpr int Y4_GPIO_NUM    = 19;
constexpr int Y3_GPIO_NUM    = 18;
constexpr int Y2_GPIO_NUM    = 5;
constexpr int VSYNC_GPIO_NUM = 25;
constexpr int HREF_GPIO_NUM  = 23;
constexpr int PCLK_GPIO_NUM  = 22;

httpd_handle_t stream_httpd = nullptr;

// MJPEG HTTP Stream Handler
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = nullptr;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      size_t hlen = snprintf(part_buf, sizeof(part_buf),
                             "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                             fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, "\r\n", 2);
      }
      esp_camera_fb_return(fb);
    }
    if (res != ESP_OK) break;
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = nullptr
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA; // 640x480 resolution
  config.jpeg_quality = 12;            // 0-63 (lower = higher quality)
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.setSleep(false);

  startCameraServer();
  Serial.print("\nAccess Point Ready! Connect to Wi-Fi: ");
  Serial.println(AP_SSID);
  Serial.print("Open in browser: http://");
  Serial.println(WiFi.softAPIP()); // Default IP is 192.168.4.1
}

void loop() {
  delay(10000);
}