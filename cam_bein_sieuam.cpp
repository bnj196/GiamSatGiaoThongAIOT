#include <Arduino.h>

// Định nghĩa các chân kết nối theo đúng cấu hình hệ thống
const int TRIG_PIN = 1;  // Chân Trig của SRF05 nối với GPIO1
const int ECHO_PIN = 2;  // Chân Echo của SRF05 nối với GPIO2

// Biến lưu trữ dữ liệu
long duration;
float distance;
float filteredDistance;

// Cấu hình bộ lọc Moving Average (Lọc trung bình động) để tránh nhiễu tín hiệu
const int SAMPLE_SIZE = 5;
float readings[SAMPLE_SIZE];
int readIndex = 0;
float total = 0;

void setup() {
  // Khởi tạo Serial Monitor với tốc độ baudrate chuẩn cho ESP32-S3
  Serial.begin(115200);
  while (!Serial) {
    ; // Đợi cổng Serial kết nối (chỉ dùng khi debug qua cáp USB trực tiếp)
  }

  // Cấu hình chức năng các chân GPIO
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Đưa chân Trig về mức thấp ban đầu
  digitalWrite(TRIG_PIN, LOW);
  
  // Khởi tạo mảng lọc nhiễu bằng 0
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    readings[i] = 0;
  }

  Serial.println("=========================================");
  Serial.println("ESP32-S3 & SRF05 ULTRASONIC SENSOR TEST");
  Serial.println("Config: Trig -> GPIO1 | Echo -> GPIO2");
  Serial.println("=========================================");
}

void loop() {
  // 1. Phát xung kích hoạt (Trigger Pulse) tối thiểu 10 micro giây
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 2. Đo thời gian xung High xuất hiện tại chân Echo (tính bằng micro giây)
  // Timeout được cấu hình là 26000us (~450cm) để tránh treo mạch khi không có vật cản
  duration = pulseIn(ECHO_PIN, HIGH, 26000);

  // 3. Tính toán khoảng cách (vận tốc âm thanh ~343m/s ở 20 độ C)
  // Khoảng cách = (Thời gian * Vận tốc âm thanh) / 2 (do sóng đi và về)
  // Công thức rút gọn: distance = duration / 58.2 (đơn vị: cm)
  if (duration == 0) {
    distance = -1; // Trả về -1 nếu vượt quá tầm đo hoặc cảm biến không phản hồi
  } else {
    distance = (float)duration / 58.2;
  }

  // 4. Áp dụng bộ lọc trung bình động nếu dữ liệu hợp lệ
  if (distance > 0) {
    total = total - readings[readIndex];
    readings[readIndex] = distance;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    if (readIndex >= SAMPLE_SIZE) {
      readIndex = 0;
    }
    filteredDistance = total / SAMPLE_SIZE;
  }

  // 5. In kết quả lên Serial Monitor và Serial Plotter để theo dõi trực quan
  if (distance == -1) {
    Serial.println("[CẢNH BÁO]: Ngoài phạm vi đo hoặc lỗi kết nối phần cứng!");
  } else {
    // Định dạng log dạng Key-Value giúp vừa đọc chữ vừa vẽ được biểu đồ trên Serial Plotter
    Serial.print("Gốc_cm:");
    Serial.print(distance);
    Serial.print(" , ");
    Serial.print("Bộ_lọc_cm:");
    Serial.println(filteredDistance);
  }

  // Tần suất quét: 60ms một lần (tốc độ lấy mẫu tiêu chuẩn tránh nhiễu sóng vọng)
  delay(60);
}
