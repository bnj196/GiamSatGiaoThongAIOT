#include <Arduino.h>
#include "esp_camera.h"

// ================= CẤU HÌNH CHÂN CAMERA =================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       9
#define Y5_GPIO_NUM       11
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       12
#define Y2_GPIO_NUM       10
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

// Magic bytes header
const uint8_t MAGIC_HEADER[4] = {0xAA, 0xBB, 0xCC, 0xDD};

void setup() {
  Serial.begin(921600);  // Giảm baud để ổn định hơn
  
  camera_config_t config;
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
  
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.println("INIT_ERROR");
    while(true) { delay(1000); }
  }
}

void loop() {
  static uint32_t lastFrameTime = 0;
  
  // Giới hạn FPS thực sự (~30 FPS)
  if (millis() - lastFrameTime < 33) {
    yield();
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    yield();
    return;
  }

  // Kiểm tra định dạng
  if (fb->format != PIXFORMAT_JPEG) {
    esp_camera_fb_return(fb);
    return;
  }

  // Gửi header + length + data
  Serial.write(MAGIC_HEADER, 4);
  Serial.flush();  // Đảm bảo header được gửi trước
  
  uint32_t img_len = fb->len;
  Serial.write((uint8_t*)&img_len, 4);
  Serial.flush();
  
  Serial.write(fb->buf, fb->len);
  Serial.flush();

  esp_camera_fb_return(fb);
  
  lastFrameTime = millis();
}