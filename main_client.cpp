#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h> 
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"




// ================= CẤU HÌNH CHÂN PHẦN CỨNG =================

#define TRIG1_PIN 12 // Cảm biến 1 (Vạch 1)
#define ECHO1_PIN 13


#define TRIG2_PIN 14 // Cảm biến 2 (Vạch 2)
#define ECHO2_PIN 15


#define LED_GREEN_1 2 // Đèn LED Cổng 1 & 2
#define LED_RED_1   4
#define LED_GREEN_2 5
#define LED_RED_2   6

// Ngưỡng khoảng cách phát hiện xe (đơn vị: cm)
#define DISTANCE_THRESHOLD 10.0 




// ================= CẤU HÌNH MQTT SERVER =================
const char* mqtt_server = "192.168.1.100"; // THAY BẰNG IP CỦA MÁY TÍNH CHẠY PYTHON SERVER
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// Biến chống dội (Debounce) để không gửi liên tục 1 xe
unsigned long lastTriggerTime1 = 0;
unsigned long lastTriggerTime2 = 0;
const int triggerDelay = 2000; // Nghỉ 2 giây sau khi bắt được 1 xe





// ================= ĐIỀN WIFI NHÀ BẠN =================
const char *ssid = "NH K7 P408-2.4G"; 
const char *password = "";            
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

// Hàm khởi tạo các chân In/Out
void initHardware() {
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_RED_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
  pinMode(LED_RED_2, OUTPUT);

  // Tắt toàn bộ đèn ban đầu
  digitalWrite(LED_GREEN_1, LOW);
  digitalWrite(LED_RED_1, LOW);
  digitalWrite(LED_GREEN_2, LOW);
  digitalWrite(LED_RED_2, LOW);
}

// Hàm đo khoảng cách của HC-SR05
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Đọc thời gian xung vọng lại (timeout 30000us ~ 5m để không bị treo)
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0; // Không thấy vật cản
  
  return (duration * 0.0343) / 2.0; // Tính ra cm
}


// Hàm xử lý khi nhận được lệnh bật đèn từ Server
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Nhận lệnh từ Server: " + message);

  // Logic đơn giản: Server gửi "VIOLATION" hoặc "NORMAL"
  if (message == "VIOLATION") {
    digitalWrite(LED_RED_2, HIGH);
    digitalWrite(LED_GREEN_2, LOW);
  } else if (message == "NORMAL") {
    digitalWrite(LED_GREEN_2, HIGH);
    digitalWrite(LED_RED_2, LOW);
  }
}

// Hàm giữ kết nối MQTT
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Đang kết nối MQTT...");
    if (client.connect("ESP32_Camera_Node")) {
      Serial.println(" Thành công!");
      client.subscribe("server/led_control"); // Lắng nghe lệnh điều khiển đèn
    } else {
      Serial.print(" Lỗi, mã rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}


// Task chạy ngầm chuyên xử lý Sensor và MQTT
void sensorTask(void * parameter) {
  for(;;) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop(); // Duy trì kết nối MQTT

    unsigned long currentTime = millis();

    // 1. Kiểm tra Vạch 1
    if (currentTime - lastTriggerTime1 > triggerDelay) {
      float dist1 = getDistance(TRIG1_PIN, ECHO1_PIN);
      if (dist1 < DISTANCE_THRESHOLD) {
        Serial.printf("VẠCH 1: Có xe! Khoảng cách: %.1f cm\n", dist1);
        
        // Tạo chuỗi JSON gửi đi
        char msg[50];
        snprintf(msg, 50, "{\"gate\":1, \"timestamp\":%lu}", currentTime);
        client.publish("sensor/trigger", msg);
        
        lastTriggerTime1 = currentTime;
      }
    }

    // 2. Kiểm tra Vạch 2
    if (currentTime - lastTriggerTime2 > triggerDelay) {
      float dist2 = getDistance(TRIG2_PIN, ECHO2_PIN);
      if (dist2 < DISTANCE_THRESHOLD) {
        Serial.printf("VẠCH 2: Có xe! Khoảng cách: %.1f cm\n", dist2);
        
        char msg[50];
        snprintf(msg, 50, "{\"gate\":2, \"timestamp\":%lu}", currentTime);
        client.publish("sensor/trigger", msg);
        
        lastTriggerTime2 = currentTime;
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Quét mỗi 50ms để không chiếm dụng CPU
  }
}



void setup()
{
  Serial.begin(115200); 
  Serial.setDebugOutput(true);
  Serial.println("\n\n-------- KHỞI ĐỘNG HỆ THỐNG ------------");
  initHardware();

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


  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

// Tạo một Task riêng cho Sensor chạy trên Core 1 (Core 0 chạy Camera và WiFi)
  xTaskCreatePinnedToCore(
    sensorTask,   // Tên hàm Task
    "SensorTask", // Tên hiển thị (để debug)
    4096,         // Kích thước RAM (Stack)
    NULL,         // Tham số truyền vào
    1,            // Mức ưu tiên (Priority)
    NULL,         // Handle
    1             // Chạy trên Core 1
  );


  Serial.println("\n============ HƯỚNG DẪN CHẠY PYTHON ============");
  Serial.printf("Copy dòng dưới đây dán vào Terminal của máy tính:\n\n");
  Serial.printf("python D:\\CURSOR\\iot\\demo_CAM.py --weights D:\\CURSOR\\iot\\runs\\detect\\runs\\vehicle_detection-16\\weights\\best.pt --source http://%s:81/stream --conf 0.55\n", ip.toString().c_str());
  Serial.println("\n===============================================");
}

void loop() {
  delay(10000);
}
