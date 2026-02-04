import serial
import struct
import threading
import time
import collections
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# 1. Cấu hình kết nối
PORT = '/dev/ttyACM0'       # Thay bằng 'COMx' nếu dùng Windows
BAUD_RATE = 115200

# 2. Cấu hình hiển thị dữ liệu
MAX_POINTS = 5000            # Số lượng điểm dữ liệu hiển thị trên màn hình (Độ rộng cửa sổ thời gian)
PLOT_INTERVAL = 50          # Tốc độ làm mới đồ thị (ms)

# 3. Cấu hình giới hạn trục Y (Y-Axis Limits)
# Để None nếu muốn tự động co giãn (Auto-scale)
# Để [min, max] nếu muốn cố định giá trị
Auto_y = 1
# Đồ thị 1: Body State (Góc thân & Vận tốc góc)
Y_LIM_PLOT_1 = [-0.5, 0.5]  

# Đồ thị 2: Wheel State (Góc bánh xe & Vận tốc góc bánh xe)
Y_LIM_PLOT_2 = None         # Auto-scale cho bánh xe

# Đồ thị 3: Float[5] (Giá trị thử nghiệm)
Y_LIM_PLOT_3 = [-5.0, 5.0]   

# Cấu trúc gói tin: Start(1B) + 5 float(20B) + Checksum(1B) = 22 Bytes
STRUCT_FORMAT = '<B5fB'  
PACKET_SIZE = struct.calcsize(STRUCT_FORMAT)

# Bộ đệm dữ liệu
data_buffer = {
    'time': collections.deque(maxlen=MAX_POINTS),
    'angular': collections.deque(maxlen=MAX_POINTS),    # Float 1
    'gyroRate': collections.deque(maxlen=MAX_POINTS),   # Float 2
    'wheelPitch': collections.deque(maxlen=MAX_POINTS), # Float 3
    'wheelRate': collections.deque(maxlen=MAX_POINTS),  # Float 4
    'float5': collections.deque(maxlen=MAX_POINTS)      # Float 5
}

lock = threading.Lock()
running = True

def serial_reader():
    """Luồng đọc dữ liệu từ cổng Serial"""
    global running
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        print(f"[INFO] Đã kết nối {PORT} tại {BAUD_RATE}")
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[ERROR] Không thể mở cổng Serial: {e}")
        running = False
        return

    while running:
        try:
            while ser.in_waiting > 0:
                # Tìm byte bắt đầu 0xAA
                byte = ser.read(1)
                if byte == b'\xAA':
                    # Đọc nốt phần còn lại
                    remaining_bytes = ser.read(PACKET_SIZE - 1)
                    
                    if len(remaining_bytes) == PACKET_SIZE - 1:
                        full_packet = b'\xAA' + remaining_bytes
                        
                        # Kiểm tra Checksum (XOR)
                        calc_checksum = 0
                        for b in full_packet[:-1]: 
                            calc_checksum ^= b
                        
                        unpacked = struct.unpack(STRUCT_FORMAT, full_packet)
                        recv_checksum = unpacked[6]

                        if calc_checksum == recv_checksum:
                            values = unpacked[1:6] # Lấy 5 giá trị float
                            
                            with lock:
                                current_time = time.time()
                                data_buffer['time'].append(current_time)
                                data_buffer['angular'].append(values[0])
                                data_buffer['gyroRate'].append(values[1])
                                data_buffer['wheelPitch'].append(values[2])
                                data_buffer['wheelRate'].append(values[3])
                                data_buffer['float5'].append(values[4])
                        else:
                            print(f"[WARN] Checksum sai! Tính: {calc_checksum}, Nhận: {recv_checksum}")
                    else:
                        # Gói tin bị đứt đoạn
                        pass
        except Exception as e:
            print(f"[ERROR] Mất kết nối Serial: {e}")
            break
            
    ser.close()
    print("[INFO] Đã đóng cổng Serial")

def update_plot(frame):
    """Hàm cập nhật giao diện đồ thị"""
    with lock:
        if len(data_buffer['time']) < 2:
            return
        
        # Chuẩn hóa thời gian về 0
        t = list(data_buffer['time'])
        start_t = t[0]
        t = [x - start_t for x in t]
        
        d_angular = list(data_buffer['angular'])
        d_gyro = list(data_buffer['gyroRate'])
        d_w_pitch = list(data_buffer['wheelPitch'])
        d_w_rate = list(data_buffer['wheelRate'])
        d_float5 = list(data_buffer['float5'])

    # --- VẼ ĐỒ THỊ 1: Thân xe ---
    ax1.cla()
    ax1.plot(t, d_angular, label='Angular (rad)', color='blue', linewidth=1.5)
    ax1.plot(t, d_gyro, label='Gyro Rate (rad/s)', color='cyan', linestyle='--', linewidth=1)
    ax1.set_ylabel('Body State')
    ax1.legend(loc='upper left', fontsize='small')
    ax1.grid(True, linestyle=':', alpha=0.6)
    if Auto_y:
        if Y_LIM_PLOT_1: ax1.set_ylim(Y_LIM_PLOT_1)

    # --- VẼ ĐỒ THỊ 2: Bánh xe ---
    ax2.cla()
    ax2.plot(t, d_w_pitch, label='Wheel Pitch (rad)', color='green', linewidth=1.5)
    ax2.plot(t, d_w_rate, label='Wheel Rate (rad/s)', color='lime', linestyle='--', linewidth=1)
    ax2.set_ylabel('Wheel State')
    ax2.legend(loc='upper left', fontsize='small')
    ax2.grid(True, linestyle=':', alpha=0.6)
    if Auto_y:
        if Y_LIM_PLOT_2: ax2.set_ylim(Y_LIM_PLOT_2)

    # --- VẼ ĐỒ THỊ 3: Giá trị thử nghiệm ---
    ax3.cla()
    ax3.plot(t, d_float5, label='control value', color='red', linewidth=1.5)
    ax3.set_ylabel('rad/s') # Không có đơn vị cụ thể
    ax3.set_xlabel('Time (s)')
    ax3.legend(loc='upper left', fontsize='small')
    ax3.grid(True, linestyle=':', alpha=0.6)
    if Auto_y:
        if Y_LIM_PLOT_3: ax3.set_ylim(Y_LIM_PLOT_3)

if __name__ == "__main__":
    # Khởi tạo giao diện 3 đồ thị dọc
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.suptitle('Real-time Robot Telemetry', fontsize=14)
    
    # Khởi tạo luồng đọc Serial
    thread = threading.Thread(target=serial_reader)
    thread.daemon = True
    thread.start()
    
    # Chạy animation
    ani = FuncAnimation(fig, update_plot, interval=PLOT_INTERVAL)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    
    running = False
    thread.join()