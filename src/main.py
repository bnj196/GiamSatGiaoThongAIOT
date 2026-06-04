#!/usr/bin/env python3
"""
ESP32-CAM Serial Stream Viewer
Đọc luồng JPEG từ Serial và hiển thị bằng OpenCV
"""

import serial
import serial.tools.list_ports
import struct
import cv2
import numpy as np
import sys
import time


# ==================== CẤU HÌNH ====================
MAGIC_HEADER = bytes([0xAA, 0xBB, 0xCC, 0xDD])
BAUD_RATE = 921600          # Phải khớp với Arduino
TIMEOUT = 2                 # Giây
MAX_FRAME_SIZE = 200 * 1024  # 200KB max (tránh crash nếu sync sai)


def find_esp32_port():
    """Tự động tìm cổng Serial của ESP32"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # ESP32 thường có description chứa "USB" hoặc "UART"
        if any(keyword in port.description for keyword in 
               ["USB", "UART", "CP210", "CH340", "FTDI"]):
            print(f"[INFO] Tìm thấy thiết bị: {port.device} - {port.description}")
            return port.device
    return None


class CameraStreamReader:
    def __init__(self, port=None, baud=BAUD_RATE):
        self.port = port or find_esp32_port()
        if not self.port:
            raise RuntimeError("Không tìm thấy cổng Serial! Hãy kiểm tra kết nối.")
        
        self.baud = baud
        self.ser = None
        self.frame_count = 0
        self.fps_time = time.time()
        self.current_fps = 0
        
    def connect(self):
        """Mở kết nối Serial"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT
            )
            print(f"[OK] Đã kết nối {self.port} @ {self.baud} baud")
            # Xóa buffer cũ
            self.ser.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print(f"[LỖI] Không thể mở cổng {self.port}: {e}")
            return False
    
    def sync_stream(self):
        """Đồng bộ lại với magic header khi bị lệch"""
        print("[INFO] Đang đồng bộ stream...")
        buffer = bytearray()
        
        while True:
            byte = self.ser.read(1)
            if not byte:
                continue
            buffer.extend(byte)
            
            # Giữ buffer tối đa 8 byte
            if len(buffer) > 8:
                buffer = buffer[-8:]
            
            # Tìm magic header
            if len(buffer) >= 4 and bytes(buffer[-4:]) == MAGIC_HEADER:
                print("[OK] Đã đồng bộ!")
                return True
    
    def read_frame(self):
        """
        Đọc 1 frame JPEG từ Serial
        Protocol: [4 bytes Magic] + [4 bytes Length] + [N bytes JPEG]
        """
        # Đọc 4 byte magic header
        magic = self.ser.read(4)
        if len(magic) != 4:
            return None
            
        if magic != MAGIC_HEADER:
            print(f"[WARN] Magic header không khớp: {magic.hex()}")
            self.sync_stream()
            return None
        
        # Đọc 4 byte length (uint32 little-endian)
        len_bytes = self.ser.read(4)
        if len(len_bytes) != 4:
            return None
        
        img_len = struct.unpack('<I', len_bytes)[0]
        
        # Kiểm tra kích thước hợp lệ
        if img_len == 0 or img_len > MAX_FRAME_SIZE:
            print(f"[WARN] Kích thước frame không hợp lệ: {img_len}")
            self.sync_stream()
            return None
        
        # Đọc dữ liệu ảnh
        img_data = self.ser.read(img_len)
        if len(img_data) != img_len:
            print(f"[WARN] Thiếu dữ liệu: {len(img_data)}/{img_len}")
            return None
        
        # Kiểm tra JPEG header (0xFF 0xD8) và footer (0xFF 0xD9)
        if not (img_data[0] == 0xFF and img_data[1] == 0xD8):
            print("[WARN] Không phải JPEG hợp lệ")
            return None
        
        return img_data
    
    def decode_and_show(self, img_data):
        """Decode JPEG và hiển thị bằng OpenCV"""
        # Chuyển bytes thành numpy array
        np_arr = np.frombuffer(img_data, dtype=np.uint8)
        
        # Decode JPEG
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return False
        
        # Tính FPS
        self.frame_count += 1
        elapsed = time.time() - self.fps_time
        if elapsed >= 1.0:
            self.current_fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_time = time.time()
        
        # Overlay thông tin
        h, w = frame.shape[:2]
        info_text = f"Resolution: {w}x{h} | FPS: {self.current_fps:.1f}"
        cv2.putText(frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Hiển thị
        cv2.imshow("ESP32-CAM Stream", frame)
        return True
    
    def run(self):
        """Vòng lặp chính"""
        if not self.connect():
            return
        
        print("\n[INFO] Nhấn 'q' để thoát, 's' để chụp ảnh\n")
        
        # Đợi ESP32 khởi động
        time.sleep(2)
        self.sync_stream()
        
        try:
            while True:
                img_data = self.read_frame()
                
                if img_data:
                    self.decode_and_show(img_data)
                
                # Kiểm tra phím bấm (chờ 1ms)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[INFO] Thoát...")
                    break
                elif key == ord('s'):
                    # Chụp ảnh lưu file
                    filename = f"capture_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                    with open(filename, 'wb') as f:
                        f.write(img_data)
                    print(f"[OK] Đã lưu: {filename}")
                    
        except KeyboardInterrupt:
            print("\n[INFO] Ngắt bởi người dùng")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Dọn dẹp tài nguyên"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        cv2.destroyAllWindows()
        print("[OK] Đã đóng kết nối")


# ==================== CHẾ ĐỘ WEB SERVER (Tùy chọn) ====================

from flask import Flask, Response
import threading

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()


def generate_frames():
    """Generator cho MJPEG stream HTTP"""
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.033)  # ~30 FPS


@app.route('/')
def index():
    return '''
    <html>
    <head><title>ESP32-CAM Stream</title></head>
    <body style="margin:0;background:#000;">
        <img src="/video_feed" style="width:100%;height:100vh;object-fit:contain;">
    </body>
    </html>
    '''


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


class WebStreamServer:
    """Kết hợp Serial reader + Flask web server"""
    
    def __init__(self, port=None, baud=BAUD_RATE):
        self.reader = CameraStreamReader(port, baud)
        self.running = False
        
    def serial_reader_thread(self):
        """Thread đọc Serial liên tục"""
        if not self.reader.connect():
            return
        
        time.sleep(2)
        self.reader.sync_stream()
        
        global latest_frame
        while self.running:
            img_data = self.reader.read_frame()
            if img_data:
                with frame_lock:
                    latest_frame = img_data
                    
    def run(self, host='0.0.0.0', web_port=5000):
        """Chạy cả Serial reader và Web server"""
        self.running = True
        
        # Khởi động thread đọc Serial
        serial_thread = threading.Thread(target=self.serial_reader_thread)
        serial_thread.daemon = True
        serial_thread.start()
        
        print(f"\n[INFO] Web server: http://{host}:{web_port}")
        print("[INFO] Truy cập từ điện thoại/tablet trong cùng mạng WiFi\n")
        
        try:
            app.run(host=host, port=web_port, threaded=True)
        except KeyboardInterrupt:
            self.running = False
            self.reader.cleanup()


# ==================== MAIN ====================

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='ESP32-CAM Serial Viewer')
    parser.add_argument('--port', '-p', help='Cổng Serial (auto-detect nếu không chỉ định)')
    parser.add_argument('--baud', '-b', type=int, default=BAUD_RATE, help='Baud rate')
    parser.add_argument('--web', '-w', action='store_true', help='Chạy ở chế độ web server')
    parser.add_argument('--web-port', type=int, default=5000, help='Port web server')
    
    args = parser.parse_args()
    
    if args.web:
        # Chế độ web server
        server = WebStreamServer(port=args.port, baud=args.baud)
        server.run(web_port=args.web_port)
    else:
        # Chế độ hiển thị trực tiếp (mặc định)
        viewer = CameraStreamReader(port=args.port, baud=args.baud)
        viewer.run()