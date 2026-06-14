
import argparse
import time
import threading
import json
import queue
import socket
from typing import Optional, Union, Dict, List, Tuple

import cv2
import numpy as np
import requests
from ultralytics import YOLO

# ================= CẤU HÌNH HỆ THỐNG =================
ESP32_STREAM_DEFAULT = "http://192.168.4.1:81/stream" 

# --- Cấu hình Mạng UDP LAN ---
UDP_LISTEN_PORT = 5005  
ESP32_UDP_PORT = 5006   

DISTANCE_BETWEEN_GATES = 16.3  # cm
SPEED_LIMIT = 6.0              # cm 
DISPLAY_DELAY_MS = 200.0       

# --- Cấu hình 4 Tầng Logic (Data Association) ---git 
MIN_TRAVEL_TIME_MS = 20.0     
MAX_TRAVEL_TIME_MS = 15000.0   

# === VỊ TRÍ 2 VẠCH CẢM BIẾN NGANG (Trục Y) ===
# Tính theo tỷ lệ % chiều cao khung hình (0.0 là đỉnh, 1.0 là đáy)
Y_RATIO_GATE_1 = 0.35  # Vạch S1 nằm ở 35% chiều cao ảnh
Y_RATIO_GATE_2 = 0.85  # Vạch S2 nằm ở 85% chiều cao ảnh

WINDOW_NAME = "Central AI Server - 4-Tier (AI Floor Filter)"
# =====================================================

class Esp32MjpegReader:
    def __init__(self, url: str, timeout: float = 5.0):
        self.url = url
        self.timeout = timeout
        self._session: Optional[requests.Session] = None
        self._stream = None
        self._buffer = bytes()

    def _connect(self) -> bool:
        self.close()
        self._session = requests.Session()
        try:
            resp = self._session.get(self.url, stream=True, timeout=self.timeout)
            resp.raise_for_status()
            self._stream = resp
            self._buffer = bytes()
            return True
        except requests.RequestException:
            self.close()
            return False

    def read(self) -> tuple[bool, Optional[np.ndarray], float]:
        if self._stream is None and not self._connect():
            return False, None, 0.0
        try:
            for chunk in self._stream.iter_content(chunk_size=4096):
                if not chunk: continue
                self._buffer += chunk
                start = self._buffer.find(b"\xff\xd8")
                end = self._buffer.find(b"\xff\xd9")
                
                if start == -1 or end == -1 or end <= start: continue

                header_data = self._buffer[:start].decode('utf-8', errors='ignore')
                frame_ts = time.time() * 1000 
                
                for line in header_data.split('\r\n'):
                    if line.startswith("X-Timestamp:"):
                        ts_str = line.split("X-Timestamp:")[1].strip()
                        try:
                            sec, usec = ts_str.split('.')
                            frame_ts = int(sec) * 1000 + int(usec) / 1000.0
                        except ValueError:
                            pass
                        break
                        
                jpg = self._buffer[start : end + 2]
                self._buffer = self._buffer[end + 2 :]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None: 
                    return True, frame, frame_ts
                    
        except requests.RequestException:
            self.close()
        return False, None, 0.0

    def close(self) -> None:
        if self._stream: self._stream.close(); self._stream = None
        if self._session: self._session.close(); self._session = None
        self._buffer = bytes()

class ThreadedCamera:
    def __init__(self, source: Union[int, str]):
        self.reader = Esp32MjpegReader(source) if (isinstance(source, str) and source.startswith("http")) else cv2.VideoCapture(source)
        self.use_cv2 = not isinstance(self.reader, Esp32MjpegReader)
        self.frame, self.status, self.running, self.frame_ts = None, False, False, 0.0
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()
        time.sleep(0.5)
        return self

    def update(self):
        while self.running:
            if self.use_cv2:
                ok, frame = self.reader.read()
                frame_ts = time.time() * 1000 
            else:
                ok, frame, frame_ts = self.reader.read()
                
            if ok and frame is not None:
                with self.lock:
                    self.frame = frame
                    self.status = True
                    self.frame_ts = frame_ts
            else:
                # TỐI ƯU: Đã giảm sleep từ 0.05 xuống 0.01 để luồng video bắt frame nhạy hơn
                time.sleep(0.01)

    def read(self):
        with self.lock: return self.status, self.frame, self.frame_ts

    def stop(self):
        self.running = False
        if self.use_cv2: self.reader.release()
        else: self.reader.close()

# ================= TIME-RING BUFFER =================
class FrameBuffer:
    def __init__(self, max_size=45): 
        self.buffer: List[Dict] = []
        self.max_size = max_size
        
    def add(self, timestamp: float, frame: np.ndarray, tracks: List[Dict]):
        if len(self.buffer) >= self.max_size:
            self.buffer.pop(0)
        self.buffer.append({'timestamp': timestamp, 'frame': frame, 'tracks': tracks})

    def get_bounding_frames(self, target_ts: float, tolerance: float = 300.0) -> Tuple[Optional[Dict], Optional[Dict]]:
        if not self.buffer: return None, None
        
        sorted_buf = sorted(self.buffer, key=lambda x: x['timestamp'])
        frame_before = None
        frame_after = None

        for i in range(len(sorted_buf) - 1):
            if sorted_buf[i]['timestamp'] <= target_ts <= sorted_buf[i+1]['timestamp']:
                frame_before = sorted_buf[i]
                frame_after = sorted_buf[i+1]
                break

        if not frame_before and not frame_after:
            closest = min(self.buffer, key=lambda x: abs(x['timestamp'] - target_ts))
            if abs(closest['timestamp'] - target_ts) <= tolerance:
                return closest, closest 
            return None, None

        return frame_before, frame_after

    def get_delayed_display_frame(self, delay_ms: float) -> Optional[Dict]:
        if not self.buffer: return None
        target_ts = self.buffer[-1]['timestamp'] - delay_ms
        return min(self.buffer, key=lambda x: abs(x['timestamp'] - target_ts))


# ================= SERVER CORE =================
class VehicleRecord:
    def __init__(self, track_id: int, lane_id: int):
        self.track_id = track_id
        self.lane_id = lane_id
        self.t1: float = 0.0 
        self.t2: float = 0.0 
        self.speed: float = 0.0
        self.is_violation: bool = False
        self.completed: bool = False
        self.y_at_s1: float = 0.0  

class CentralServer:
    def __init__(self, weights: str, source: str):
        self.model = YOLO(weights)
        self.stream = ThreadedCamera(source).start()
        
        self.sensor_queue = queue.Queue()
        self.vehicles: Dict[int, VehicleRecord] = {}
        self.history_buffer = FrameBuffer(max_size=45) 
        
        self.esp32_ip = None 
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # TỐI ƯU: Cấp phát 1MB bộ đệm để UDP không bị rớt gói khi YOLO đang bận tính toán
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.udp_sock.bind(("0.0.0.0", UDP_LISTEN_PORT))
        
        self.is_running = True
        threading.Thread(target=self.udp_listener_loop, daemon=True).start()
        print(f"[NET] Đã mở Local UDP Socket. Mở rộng Buffer. Đang lắng nghe Port {UDP_LISTEN_PORT}...")

    def udp_listener_loop(self):
        while self.is_running:
            try:
                data, addr = self.udp_sock.recvfrom(1024)
                if self.esp32_ip != addr[0]:
                    self.esp32_ip = addr[0]
                    print(f"[NET] Đã nhận diện IP của ESP32: {self.esp32_ip}")

                payload = json.loads(data.decode('utf-8'))
                self.sensor_queue.put(payload)
            except json.JSONDecodeError:
                pass
            except Exception as e:
                if self.is_running: print(f"[NET] UDP Error: {e}")

    def send_udp_command(self, command: str):
        if self.esp32_ip:
            try:
                self.udp_sock.sendto(command.encode('utf-8'), (self.esp32_ip, ESP32_UDP_PORT))
            except Exception:
                pass

    def assign_lane(self, x_center: float, frame_width: float) -> int:
        ratio = x_center / frame_width
        if ratio < 0.33: return 1
        elif ratio < 0.66: return 2
        else: return 3

    def interpolate_tracks(self, frame_before: Dict, frame_after: Dict, target_ts: float) -> List[Dict]:
        if frame_before == frame_after: return frame_before['tracks']
        t_A, t_B = frame_before['timestamp'], frame_after['timestamp']
        if t_B == t_A: return frame_before['tracks']
        
        ratio = (target_ts - t_A) / (t_B - t_A)
        tracks_A = {t['id']: t for t in frame_before['tracks']}
        tracks_B = {t['id']: t for t in frame_after['tracks']}
        
        virtual_tracks = []
        for tid in set(tracks_A.keys()).union(set(tracks_B.keys())):
            if tid in tracks_A and tid in tracks_B:
                xA, xB = tracks_A[tid]['xc'], tracks_B[tid]['xc']
                yA, yB = tracks_A[tid].get('yc', 0), tracks_B[tid].get('yc', 0)
                virtual_tracks.append({
                    'id': tid, 
                    'xc': xA + (xB - xA) * ratio,
                    'yc': yA + (yB - yA) * ratio,
                    'box': tracks_A[tid]['box'],
                    'cls': tracks_A[tid].get('cls', 'Unknown')
                })
            else:
                virtual_tracks.append(tracks_A[tid] if tid in tracks_A else tracks_B[tid])
        return virtual_tracks

    # ==================== TẦNG 1: VISUAL VERIFICATION (AI FILTER) ====================
    def verify_visual_at_gate(self, track: Dict, target_y: float, frame_height: float, gate: int) -> bool:
        """
        AI Trọng Tài: Phân biệt 'Nhiễu mặt sàn' và 'Xe thực sự chạm vạch'
        """
        x1, y1, x2, y2 = track['box']
        
        # Biên độ sai số RẤT NHỎ (2% khung hình ~ 10px) 
        # Để đảm bảo xe phải CÁN đúng vạch mới tính, tránh nhận vơ từ xa
        margin = frame_height * 0.02 
        
        # ĐIỀU KIỆN TIÊN QUYẾT: Vạch ngang (target_y) phải cắt qua thân xe
        if (y1 - margin) <= target_y <= (y2 + margin):
            return True
            
        # NẾU KHÔNG CẮT QUA: Khung hình S2 lúc này chỉ là mặt sàn (cảm biến báo nhiễu)
        dist = target_y - y2 if target_y > y2 else y1 - target_y
        print(f"[AI-FILTER] Bỏ qua S{gate}: Cảm biến báo ảo. Xe chưa chạm vạch (Còn {dist:.1f}px)")
        return False

    # ==================== TẦNG 2: SPATIAL CONSTRAINT ====================
    def verify_spatial_consistency(self, record: VehicleRecord, current_track: Dict, 
                                   frame_width: float) -> bool:
        """Kiểm tra lấn làn và chiều di chuyển dọc (Y-axis)"""
        current_lane = self.assign_lane(current_track['xc'], frame_width)
        
        if record.lane_id != current_lane:
            print(f"[SPATIAL-REJECT] ID {record.track_id}: Lấn làn từ {record.lane_id} sang {current_lane}")
            return False
            
        direction_expected = np.sign(Y_RATIO_GATE_2 - Y_RATIO_GATE_1)
        direction_actual = np.sign(current_track['yc'] - record.y_at_s1)
        
        if direction_expected != 0 and direction_actual != 0 and direction_expected != direction_actual:
            print(f"[SPATIAL-REJECT] ID {record.track_id}: Đi ngược chiều (S1 y={record.y_at_s1:.1f}, S2 y={current_track['yc']:.1f})")
            return False
            
        return True

    # ==================== TẦNG 3: TEMPORAL CONSTRAINT ====================
    def verify_temporal_validity(self, record: VehicleRecord, sensor_ts: float) -> bool:
        time_diff = sensor_ts - record.t1
        if time_diff < MIN_TRAVEL_TIME_MS:
            print(f"[TEMPORAL-REJECT] ID {record.track_id}: Quá nhanh ({time_diff:.1f}ms) -> Tín hiệu bám đuôi")
            return False
        if time_diff > MAX_TRAVEL_TIME_MS:
            print(f"[TEMPORAL-REJECT] ID {record.track_id}: Quá chậm ({time_diff:.1f}ms) -> Xóa xe rác")
            if record.track_id in self.vehicles:
                del self.vehicles[record.track_id]
            return False
        return True

    def process_sensor_events(self, frame_width: float, frame_height: float):
        y_gate_1 = frame_height * Y_RATIO_GATE_1
        y_gate_2 = frame_height * Y_RATIO_GATE_2

        while not self.sensor_queue.empty():
            event = self.sensor_queue.get()
            gate = event.get("gate")
            sensor_ts = event.get("timestamp")

            f_before, f_after = self.history_buffer.get_bounding_frames(sensor_ts)
            if not f_before: continue

            virtual_tracks = self.interpolate_tracks(f_before, f_after, sensor_ts)

            # Khởi tạo Cache Memory 
            for track in virtual_tracks:
                tid = track['id']
                if tid not in self.vehicles:
                    self.vehicles[tid] = VehicleRecord(tid, self.assign_lane(track['xc'], frame_width))

            # Tìm chiếc xe (Bounding Box) gần vạch ngang nhất
            target_y = y_gate_1 if gate == 1 else y_gate_2
            best_match_id = None
            min_dist = float('inf')
            best_track = None

            for track in virtual_tracks:
                tid = track['id']
                x1, y1, x2, y2 = track['box']
                record = self.vehicles[tid]

                if gate == 1 and record.t1 != 0: continue
                if gate == 2 and (record.t1 == 0 or record.completed): continue

                # TỐI ƯU: Đo khoảng cách từ vạch đến Mép Xe, không đo đến Tâm Xe
                if y1 <= target_y <= y2:
                    dist = 0.0 # Vạch cắt qua xe -> Độ chính xác hoàn hảo
                else:
                    dist = min(abs(target_y - y1), abs(target_y - y2))

                if dist < min_dist:
                    min_dist = dist
                    best_match_id = tid
                    best_track = track

            # Xử lý 4 Tầng Logic cho xe ứng cử viên
            if best_match_id is not None:
                record = self.vehicles[best_match_id]

                # ===== TẦNG 1: VISUAL VERIFICATION (AI Lọc Nhiễu) =====
                if not self.verify_visual_at_gate(best_track, target_y, frame_height, gate):
                    continue # Bị chặn: Đây là gói UDP báo nhiễu mặt sàn, vứt!

                # Nếu qua được bộ lọc -> Chốt thông số
                if gate == 1:
                    record.t1 = sensor_ts
                    record.lane_id = self.assign_lane(best_track['xc'], frame_width)
                    record.y_at_s1 = best_track['yc']
                    print(f"[S1-LOCK] ID {best_match_id} VÀO Làn {record.lane_id} | ts={sensor_ts:.0f}")
                
                elif gate == 2:
                    # ===== TẦNG 2 & 3 =====
                    if not self.verify_spatial_consistency(record, best_track, frame_width): continue
                    if not self.verify_temporal_validity(record, sensor_ts): continue
                    
                    # ===== TẦNG 4: HOÀN TẤT =====
                    record.t2 = sensor_ts
                    self.calculate_violation(record)
            else:
                pass # Không log Unmatched nữa để tránh rác console khi cảm biến nhá liên tục

    def calculate_violation(self, record: VehicleRecord):
        time_diff_s = (record.t2 - record.t1) / 1000.0
        if time_diff_s <= 0: return

        record.speed = DISTANCE_BETWEEN_GATES / time_diff_s
        record.completed = True
        
        print(f">>> [KẾT QUẢ] ID {record.track_id} | Làn {record.lane_id} | "
              f"Vận tốc: {record.speed:.1f} cm/s | Time: {time_diff_s:.3f}s")
        
        if record.speed > SPEED_LIMIT:
            record.is_violation = True
            self.send_udp_command("VIOLATION") 
        else:
            self.send_udp_command("NORMAL")

    def run(self):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        print("SẴN SÀNG! Đang chạy luồng Camera và lọc nhiễu UDP...")
        
        try:
            while True:
                ok, frame, current_ts = self.stream.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    continue

                h, w = frame.shape[:2]
                
                # TỐI ƯU: Đã giảm conf xuống 0.3 để bắt được các xe chạy tốc độ cao bị mờ (motion blur)
                results = self.model.track(frame, persist=True, conf=0.45, 
                                          tracker="bytetrack.yaml", verbose=False)
                
                current_tracks = []
                if results[0].boxes.id is not None:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    track_ids = results[0].boxes.id.cpu().numpy().astype(int)
                    class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
                    
                    for box, track_id, class_id in zip(boxes, track_ids, class_ids):
                        x1, y1, x2, y2 = box
                        xc, yc = (x1 + x2) / 2, (y1 + y2) / 2 
                        class_name = self.model.names[class_id]
                        current_tracks.append({
                            'id': track_id, 'xc': xc, 'yc': yc,
                            'box': (x1, y1, x2, y2), 'cls': class_name 
                        })

                self.history_buffer.add(current_ts, frame.copy(), current_tracks)
                
                # Xử lý Hàng đợi UDP 
                self.process_sensor_events(w, h)

                # Rendering UI (Hiển thị trễ để Sync với UDP)
                display_data = self.history_buffer.get_delayed_display_frame(DISPLAY_DELAY_MS)
                if display_data:
                    disp_frame = display_data['frame'].copy()
                    
                    cv2.line(disp_frame, (int(w*0.33), 0), (int(w*0.33), h), (255, 255, 255), 1)
                    cv2.line(disp_frame, (int(w*0.66), 0), (int(w*0.66), h), (255, 255, 255), 1)
                    
                    y1_render = int(h * Y_RATIO_GATE_1)
                    y2_render = int(h * Y_RATIO_GATE_2)
                    cv2.line(disp_frame, (0, y1_render), (w, y1_render), (0, 0, 255), 2)
                    cv2.putText(disp_frame, "Gate 1 (S1)", (10, y1_render - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    cv2.line(disp_frame, (0, y2_render), (w, y2_render), (0, 255, 255), 2)
                    cv2.putText(disp_frame, "Gate 2 (S2)", (10, y2_render - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    for track in display_data['tracks']:
                        tid = track['id']
                        class_name = track.get('cls', 'Unknown')
                        x1, y1, x2, y2 = track['box']
                        
                        label = f"[{class_name}] ID:{tid}"
                        color = (0, 255, 0) 
                        
                        if tid in self.vehicles:
                            rec = self.vehicles[tid]
                            if rec.completed:
                                label += f" {rec.speed:.1f}cm/s"
                                color = (0, 0, 255) if rec.is_violation else (255, 0, 0)
                            elif rec.t1 > 0:
                                label += f" L{rec.lane_id} Wait S2"
                                color = (0, 255, 255)
                                
                        cv2.rectangle(disp_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                        cv2.putText(disp_frame, label, (int(x1), int(y1)-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    cv2.putText(disp_frame, f"UDP Sync Delay: {DISPLAY_DELAY_MS}ms", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    completed_count = sum(1 for v in self.vehicles.values() if v.completed)
                    cv2.putText(disp_frame, f"Tracked: {len(self.vehicles)} | Completed: {completed_count}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    cv2.imshow(WINDOW_NAME, disp_frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.is_running = False
            self.stream.stop()
            self.udp_sock.close()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=str, required=True, help="Đường dẫn file best.pt")
    parser.add_argument("--source", type=str, default=ESP32_STREAM_DEFAULT, help="Nguồn video")
    args = parser.parse_args()

    server = CentralServer(args.weights, args.source)
    server.run()