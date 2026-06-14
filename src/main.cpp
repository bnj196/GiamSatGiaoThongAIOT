#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h> // Thay thế thư viện MQTT
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

// ================= CẤU HÌNH CHÂN PHẦN CỨNG =================
#define TRIG1_PIN 1 // Cảm biến 1 (Vạch 1)
#define ECHO1_PIN 2

#define TRIG2_PIN 41 // Cảm biến 2 (Vạch 2)
#define ECHO2_PIN 42

#define LED_GREEN_1 21 // Đèn LED Cổng 1 & 2
#define LED_RED_1   47
// #define LED_GREEN_2 5
// #define LED_RED_2   6

#define DISTANCE_THRESHOLD 30

// ================= CẤU HÌNH UDP SOCKET =================
const char* server_ip = "10.3.6.117"; // IP MÁY TÍNH CHẠY PYTHON
const int server_port = 5005;         // Port máy tính lắng nghe
const int local_udp_port = 5006;      // Port ESP32 lắng nghe lệnh bật đèn

WiFiUDP udp;

unsigned long lastTriggerTime1 = 0;
unsigned long lastTriggerTime2 = 0;
const int triggerDelay = 2000; 

// ================= ĐIỀN WIFI NHÀ BẠN =================
const char *ssid = "NH K7 P408-2.4G"; 
const char *password = "";            
// =====================================================

// --- Thêm 2 biến này vào phần khai báo biến toàn cục ---
unsigned long ledOnTime = 0;   // Lưu thời điểm bật đèn
bool isLedActive = false;      // Cờ trạng thái đèn đang sáng
const int ledTimeout = 3000;   // Thời gian sáng đèn (1000ms = 1s)


void startCameraServer(); 

void initHardware() {
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_RED_1, OUTPUT);
  // pinMode(LED_GREEN_2, OUTPUT);
  // pinMode(LED_RED_2, OUTPUT);
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Giữ timeout ở mức 6000us để chống treo Camera (Camera capture failed)
  long duration = pulseIn(echoPin, HIGH, 6000);
  if (duration == 0) return 999.0; 
  
  return (duration * 0.0343) / 2.0; 
}




// Task chạy ngầm chuyên xử lý Sensor và UDP (Thế chỗ MQTT)

void sensorTask(void * parameter) {
  for(;;) {
    unsigned long currentTime = millis();

    // -------------------------------------------------------
    // 1. KIỂM TRA VẠCH 1 (GATE 1)
    // -------------------------------------------------------
    if (currentTime - lastTriggerTime1 > triggerDelay) {
      float dist1 = getDistance(TRIG1_PIN, ECHO1_PIN);
      if (dist1 < DISTANCE_THRESHOLD) {
        // Gửi chuỗi JSON qua UDP
        char msg[64];
        snprintf(msg, sizeof(msg), "{\"gate\":1, \"timestamp\":%lu}", currentTime);
        
        udp.beginPacket(server_ip, server_port);
        udp.print(msg);
        udp.endPacket();
        
        Serial.printf("[S1] Xe đi qua - Khoảng cách: %.1f cm - Đã gửi UDP\n", dist1);
        lastTriggerTime1 = currentTime;
        vTaskDelay(2 / portTICK_PERIOD_MS); // Nghỉ nhẹ để ổn định sóng âm
      }
    }

    // -------------------------------------------------------
    // 2. KIỂM TRA VẠCH 2 (GATE 2)
    // -------------------------------------------------------
    if (currentTime - lastTriggerTime2 > triggerDelay) {
      float dist2 = getDistance(TRIG2_PIN, ECHO2_PIN);
      if (dist2 < DISTANCE_THRESHOLD) {
        // Gửi chuỗi JSON qua UDP
        char msg[64];
        snprintf(msg, sizeof(msg), "{\"gate\":2, \"timestamp\":%lu}", currentTime);
        
        udp.beginPacket(server_ip, server_port);
        udp.print(msg);
        udp.endPacket();
        
        Serial.printf("[S2] Xe đi qua - Khoảng cách: %.1f cm - Đã gửi UDP\n", dist2);
        lastTriggerTime2 = currentTime;
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
    }

    // -------------------------------------------------------
    // 3. NHẬN LỆNH TỪ SERVER & ĐIỀU KHIỂN ĐÈN
    // -------------------------------------------------------
    int packetSize = udp.parsePacket();
    if (packetSize) {
      char incomingPacket[128];
      int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
      if (len > 0) {
        incomingPacket[len] = '\0';
        String message = String(incomingPacket);
        message.trim();

        if (message == "VIOLATION") {
          Serial.println(">>> KẾT QUẢ: VIOLATION (Quá tốc độ) - Bật Đèn Đỏ");
          digitalWrite(LED_RED_1, HIGH);
          digitalWrite(LED_GREEN_1, LOW);
          
          ledOnTime = millis(); // Ghi lại thời điểm bắt đầu sáng đèn
          isLedActive = true;
        } 
        else if (message == "NORMAL") {
          Serial.println(">>> KẾT QUẢ: NORMAL (Đúng tốc độ) - Bật Đèn Xanh");
          digitalWrite(LED_GREEN_1, HIGH);
          digitalWrite(LED_RED_1, LOW);
          
          ledOnTime = millis(); // Ghi lại thời điểm bắt đầu sáng đèn
          isLedActive = true;
        }
      }
    }

    // -------------------------------------------------------
    // 4. LOGIC TỰ ĐỘNG RESET ĐÈN SAU 1 GIÂY (NON-BLOCKING)
    // -------------------------------------------------------
    if (isLedActive && (currentTime - ledOnTime >= ledTimeout)) {
      digitalWrite(LED_RED_1, LOW);
      digitalWrite(LED_GREEN_1, LOW);
      isLedActive = false;
      Serial.println("[Hệ thống] Reset đèn: Đã tắt sau 1s.");
    }

    // Nghỉ 20ms mỗi chu kỳ loop để nhường CPU cho các tác vụ khác của hệ điều hành
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}

void setup()
{
  Serial.begin(115200); 
  Serial.setDebugOutput(true);
  Serial.println("\n\n-------- KHỞI ĐỘNG HỆ THỐNG ------------");
  initHardware();

  camera_config_t config;

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
  
  config.xclk_freq_hz = 17000000; 
  config.frame_size = FRAMESIZE_SVGA; 
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 13; 
  config.fb_count = 1;

  if (psramFound()) {
    config.fb_count = 2;      
    config.grab_mode = CAMERA_GRAB_LATEST; 
    Serial.println("PSRAM OK - Dùng 2 Buffer.");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("LỖI KHỞI TẠO CAMERA! Mã lỗi: 0x%x\n", err);
    return; 
  }

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

  // Khởi tạo UDP Socket lắng nghe lệnh từ Server
  udp.begin(local_udp_port);
  Serial.printf("Đã mở UDP Socket trên Port %d\n", local_udp_port);

  // Tạo một Task riêng cho Sensor chạy trên Core 0 (Cách ly khỏi Camera Stream ở Core 1)
  xTaskCreatePinnedToCore(
    sensorTask,   
    "SensorTask", 
    4096,         
    NULL,         
    1,            
    NULL,         
    0             // <-- Chạy trên Core 0
  );

  Serial.println("\n============ HƯỚNG DẪN CHẠY PYTHON ============");
  Serial.printf("Copy dòng dưới đây dán vào Terminal của máy tính:\n\n");
  Serial.printf("python D:\\CURSOR\\iot\\demo_CAM.py --weights D:\\CURSOR\\iot\\runs\\detect\\runs\\vehicle_detection-16\\weights\\best.pt --source http://%s:81/stream --conf 0.55\n", ip.toString().c_str());
  Serial.println("\n===============================================");
}

void loop() {
  delay(10000);
}
