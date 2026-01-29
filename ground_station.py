import cv2
import socket
import numpy as np
import threading
import json
import time

# ================= 配置区域 =================
JETSON_IP = '192.168.1.200' # 请修改为你的 Jetson IP
VIDEO_PORT = 9999
DATA_PORT = 8888
# ===========================================

current_gps_info = {"lat": 0, "lon": 0, "alt": 0}
tcp_socket = None  # 全局 Socket 对象
is_connected = False # 连接状态标志

# --- 新增: 倒计时发送任务线程 ---
def countdown_mission_sender():
    print("[Mission] 等待 TCP 连接建立...")

    # 简单的自旋锁，等待连接建立
    while not is_connected:
        time.sleep(1)

    print("[Mission] 连接已建立！启动 10 秒倒计时...")
    for i in range(10, 0, -1):
        print(f"[Mission] 倒计时: {i} 秒")
        time.sleep(1)

    print("[Mission] 时间到！正在发送目标坐标...")

    # 模拟一组目标航点
    target_data = {
        "type": "mission_update",
        "waypoints": [
            {"lat": 30.6600, "lon": 104.0700},
            {"lat": 30.6650, "lon": 104.0750},
            {"lat": 30.6700, "lon": 104.0800}
        ]
    }

    try:
        if tcp_socket:
            msg = json.dumps(target_data, indent=4) # 格式化一下更好看
            tcp_socket.send(msg.encode('utf-8'))
            print("[Mission] ✅ 坐标数据发送成功！")
        else:
            print("[Mission] ❌ 发送失败，Socket 未连接")
    except Exception as e:
        print(f"[Mission] 发送出错: {e}")

# --- 数据接收线程 ---
def receive_data():
    global tcp_socket, is_connected, current_gps_info

    # 创建 socket
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    print("[Data] 尝试连接无人机数据链路...")
    while True:
        try:
            tcp_socket.connect((JETSON_IP, DATA_PORT))
            print("[Data] TCP 连接成功！")
            is_connected = True # 标记连接成功
            break
        except:
            time.sleep(2) # 连接失败重试

    buffer = ""
    while True:
        try:
            data = tcp_socket.recv(1024).decode('utf-8')
            if not data: break

            buffer += data
            if "\n" in buffer:
                parts = buffer.split("\n")
                for part in parts[:-1]:
                    try:
                        info = json.loads(part)
                        current_gps_info = info
                        # 为了控制台清爽，我在视频界面已经显示了，这里就不print刷屏了
                        # print(f"Lat: {info['lat']}")
                    except: pass
                buffer = parts[-1]
        except Exception as e:
            print(f"Connection lost: {e}")
            is_connected = False
            break

# --- 视频接收主循环 ---
def receive_video():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', VIDEO_PORT))
    udp_socket.settimeout(2)

    print(f"[Video] 视频显示窗口已就绪...")

    while True:
        try:
            data, addr = udp_socket.recvfrom(65536 * 2)
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                # 叠加 GPS 信息到画面
                text = f"Lat: {current_gps_info['lat']} Lon: {current_gps_info['lon']}"
                cv2.putText(frame, text, (10, 450), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)

                # 叠加倒计时提示（可选）
                if not is_connected:
                     cv2.putText(frame, "Attempting TCP Connect...", (200, 240),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                cv2.imshow('Drone Live Stream (Windows)', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except socket.timeout:
            pass
        except Exception:
            pass

    cv2.destroyAllWindows()
    udp_socket.close()

if __name__ == '__main__':
    # 1. 启动数据接收线程
    t_recv = threading.Thread(target=receive_data)
    t_recv.daemon = True
    t_recv.start()

    # 2. 启动倒计时发送任务线程
    t_mission = threading.Thread(target=countdown_mission_sender)
    t_mission.daemon = True
    t_mission.start()

    # 3. 主线程跑视频显示
    receive_video()
