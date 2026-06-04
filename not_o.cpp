#include "esp_camera.h"
#include <WiFi.h>

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

// Hotspot do ESP32 phát — kết nối trực tiếp, không cần router
const char *ap_ssid = "ESP32-CAM";
const char *ap_password = "12345678";

void startCameraServer();

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.xclk_freq_hz = 20000000;
  // VGA: cân bằng chất lượng / FPS cho OpenCV + YOLO trên PC
  config.frame_size = FRAMESIZE_CIF;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 20;
  config.fb_count = 1;

  if (psramFound())
  {
    config.jpeg_quality = 15;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    Serial.println("PSRAM FOUND");
  }
  else
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.println("NO PSRAM FOUND");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (!WiFi.softAP(ap_ssid, ap_password))
  {
    Serial.println("SoftAP start failed");
    return;
  }

  IPAddress ip = WiFi.softAPIP();
  Serial.println();
  Serial.println("WiFi AP started");
  Serial.printf("  SSID     : %s\n", ap_ssid);
  Serial.printf("  Password : %s\n", ap_password);
  Serial.printf("  IP       : %s\n", ip.toString().c_str());

  startCameraServer();

  Serial.println();
  Serial.println("Camera ready — connect phone/PC to the AP above, then open:");
  Serial.printf("  Web UI : http://%s/\n", ip.toString().c_str());
  Serial.printf("  Stream : http://%s:81/stream\n", ip.toString().c_str());
  Serial.println();
  Serial.println("Python YOLO (PC phai ket noi WiFi ESP32-CAM):");
  Serial.printf("  python demo_CAM.py --weights best.pt --source http://%s:81/stream --conf 0.75\n",
                ip.toString().c_str());
}

void loop()
{
  delay(10000);
}

// http://192.168.4.1:81/stream