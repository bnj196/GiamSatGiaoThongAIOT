#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h> // Thư viện MQTT

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"


// ================= ĐIỀN WIFI NHÀ BẠN =================
const char *ssid = "Subin"; 
const char *password = "subinacc";            
// =====================================================

void startCameraServer(); // Hàm từ app_httpd.cpp

void checkHardwareFPS() {
  Serial.println("\n--- ĐANG KIỂM TRA TỐC ĐỘ FPS PHẦN CỨNG CAMERA ---");
  unsigned long start_time = millis();
  int frames_to_test = 50; 
  int successful_frames = 0;

  for (int i = 0; i < frames_to_test; i++) {
    camera_fb_t *fb = esp_camera_fb_get(); 
    if (fb) {
      successful_frames++;
      esp_camera_fb_return(fb); 
    } else {
      Serial.print("x "); // In dấu x nếu trượt frame
    }
  }

  unsigned long end_time = millis();
  float time_taken_s = (end_time - start_time) / 1000.0;
  float fps = (time_taken_s > 0) ? (successful_frames / time_taken_s) : 0;

  Serial.printf("\n-> Đã chụp %d frames trong %.2f giây.\n", successful_frames, time_taken_s);
  Serial.printf("-> TỐC ĐỘ CAM HARDWARE: %.2f FPS\n", fps);
  Serial.println("---------------------------------------------------\n");
}

void setup()
{
  Serial.begin(115200); 
  Serial.setDebugOutput(true);
  Serial.println("\n\n-------- KHỞI ĐỘNG HỆ THỐNG ------------");

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
  
  // HẠ XUNG NHỊP XUỐNG 14MHz ĐỂ CHỐNG LỖI 0 FPS
  config.xclk_freq_hz = 15000000; 
  config.frame_size = FRAMESIZE_CIF; 
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12; 
  config.fb_count = 1;

  if (psramFound()) {
    config.fb_count = 2;      
    config.grab_mode = CAMERA_GRAB_LATEST; 
    Serial.println("PSRAM OK - Dùng 2 Buffer.");
  }

  // Khởi tạo Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("LỖI KHỞI TẠO CAMERA! Mã lỗi: 0x%x\n", err);
    return; 
  }

  // Test Camera ngay lập tức
  // checkHardwareFPS();

  // Kết nối WiFi (Station Mode)
  WiFi.mode(WIFI_STA); 
  WiFi.setSleep(false); 
  
  Serial.printf("\nĐang kết nối vào Router WiFi: %s ", ssid);
  if (strlen(password) > 0) {
    WiFi.begin(ssid, password);
  } else {
    WiFi.begin(ssid);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  IPAddress ip = WiFi.localIP(); 
  Serial.printf("\nKết nối WiFi thành công! IP của Camera là: %s\n", ip.toString().c_str());

  startCameraServer();

  Serial.println("\n============ HƯỚNG DẪN CHẠY PYTHON ============");
  Serial.printf("Copy dòng dưới đây dán vào Terminal của máy tính:\n\n");
  Serial.printf("python D:\\CURSOR\\iot\\demo_CAM.py --weights D:\\CURSOR\\iot\\runs\\detect\\runs\\vehicle_detection-16\\weights\\best.pt --source http://%s:81/stream --conf 0.55\n", ip.toString().c_str());
  Serial.println("\n===============================================");
}

void loop() {
  delay(10000);
}