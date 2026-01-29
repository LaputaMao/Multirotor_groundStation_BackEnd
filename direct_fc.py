import math
import sys
import socket
from pymavlink import mavutil
import time
import threading
from pyproj import Proj, Transformer

#
# # ================= 配置区域 =================
# # 1. 这里填你的 USB 转 TTL 所在的端口号
# # 用于 Python 直连飞控
# FC_PORT = 'COM6'
# BAUD_RATE = 57600  # 如果连不上，试试 57600
#
# # 2. 地面站转发配置 (可选)
# # Python 把数据转发到这个地址，地面站设置UDP监听这个端口即可看到画面
# GCS_IP = '127.0.0.1'
# GCS_PORT = 14550
# # ===========================================
#
# print(f"正在尝试连接飞控串口: {FC_PORT} @ {BAUD_RATE}...")
#
# # 1. 建立与飞控的物理连接 (Serial)
# try:
#     # autoreconnect=True 会自动处理断连重连
#     master = mavutil.mavlink_connection(FC_PORT, baud=BAUD_RATE, autoreconnect=True)
#
# except Exception as e:
#     print(f"❌ 无法连接串口，请检查端口号或关闭地面站软件！\n错误: {e}")
#     exit()
#
# # 2. 建立与地面站的虚拟连接 (UDP Output)
# # source_system=1 表示我们冒充是这架飞机发出的数据
# gcs_link = mavutil.mavlink_connection(f'udpout:{GCS_IP}:{GCS_PORT}', source_system=1)
#
# print("✅连接成功！等待心跳包...")
#
#
# # --- 接收并转发线程 ---
# def connection_loop():
#     while True:
#         try:
#             # 1. 从飞控读取一条 MAVLink 消息
#             msg = master.recv_match(blocking=True)
#
#             # print(f"\n{msg}\n")
#             if not msg:
#                 continue
#
#             # 2. 把消息直接转发给地面站软件 (让他看着玩)
#             gcs_link.write(msg.get_msgbuf())
#
#             # 3. 在 Python 控制台打印感兴趣的数据
#             msg_type = msg.get_type()
#
#             if msg_type == 'HEARTBEAT':
#                 # 打印一下模式，确认连接活着
#                 print(f"[心跳] 模式: {mavutil.mode_string_v10(msg)}")
#                 print(f"\nmsg: {msg}\n")
#
#             # elif msg_type == 'ATTITUDE':
#             #     # 打印姿态
#             #     print(f"[姿态] Roll: {msg.roll:.2f} | Pitch: {msg.pitch:.2f} | Yaw: {msg.yaw:.2f}")
#             #
#             # elif msg_type == 'GLOBAL_POSITION_INT':
#             #     # 打印 GPS (如果有)
#             #     lat = msg.lat / 1e7
#             #     lon = msg.lon / 1e7
#             #     print(f"[GPS] {lat}, {lon}")
#
#         except Exception as e:
#             print(f"Error: {e}")


# NED 坐标系距离检查函数

# ================= 配置区域 =================
# ================= 配置区域 =================
FC_PORT = 'COM8'
BAUD_RATE = 57600

# 地面站配置
GCS_IP = '127.0.0.1'
GCS_PORT = 14550
# ===========================================

print(f"正在尝试连接飞控串口: {FC_PORT} @ {BAUD_RATE}...")

# 1. 建立与飞控的物理连接 (Serial) - 依然维持 pymavlink 连接以便解析数据
try:
    master = mavutil.mavlink_connection(FC_PORT, baud=BAUD_RATE, autoreconnect=True)
except Exception as e:
    print(f"❌ 无法连接串口: {e}")
    exit()

# 2. 建立与地面站的【原生UDP】连接
# 使用 socket.SOCK_DGRAM 代表 UDP
gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 设置发送目标地址 (元组形式)
gcs_addr = (GCS_IP, GCS_PORT)
# 设置非阻塞模式或超时，防止接收线程卡死 (可选)
gcs_socket.settimeout(1.0)

print(f"✅ 连接成功！透传已启动: 串口 <-> UDP({GCS_IP}:{GCS_PORT})")


# --- 线程 A: 飞控(Serial) -> 你的代码逻辑 + 地面站(UDP) ---
def fc_to_gcs_loop():
    while True:
        try:
            # 1. 从飞控读取消息 (解析后的对象)
            # blocking=True 会等待直到有消息
            msg = master.recv_match(blocking=True)

            if not msg:
                continue

            # 2. 【转发核心】获取消息的原始二进制 buffer，通过 UDP 发给地面站
            # get_msgbuf() 是 pymavlink 提供的获取该消息原始字节流的方法
            gcs_socket.sendto(msg.get_msgbuf(), gcs_addr)

            # 3. 这里的逻辑保留，给你自己看状态用
            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                # 偶尔打印一下，证明程序还活着
                # 只有当 custom_mode 变化时或者每隔一段时间打印会更好，这里保持原样
                print(f"[飞控心跳] 模式: {mavutil.mode_string_v10(msg)}")

            elif msg_type == 'COMMAND_ACK':
                print(f"📩 [指令回执] Command: {msg.command} Result: {msg.result}")

        except Exception as e:
            print(f"Serial Read Error: {e}")
            time.sleep(0.1)


# --- 线程 B: 地面站(UDP) -> 飞控(Serial) ---
def gcs_to_fc_loop():
    print("🎧 开始监听地面站指令...")
    while True:
        try:
            # 1. 监听 UDP 端口，接收来自地面站的数据
            # 4096 是缓冲区大小，通常 MAVLink 包也就 200多字节，4096足够了
            data, addr = gcs_socket.recvfrom(4096)

            if data:
                # 2. 【转发核心】直接把收到的二进制数据写入串口
                master.write(data)

                # (可选) 打印调试，看地面站发了啥命令
                # print(f"收到地面站 {len(data)} 字节数据，已转发给飞控")

        except socket.timeout:
            # UDP 接收超时是正常的（地面站可能没发指令），继续循环
            continue
        except Exception as e:
            print(f"UDP Read Error: {e}")
            time.sleep(0.1)


def wait_until_arrived(target_n, target_e, tolerance=0.5):
    """
    阻塞直到到达目标点
    tolerance: 容差范围，比如 0.5 米
    """
    print(f"正在飞往 N:{target_n}, E:{target_e} ...")
    while True:
        # 获取当前位置 需要订阅 GLOBAL_POSITION_INT 消息
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_n = msg.lon
        current_e = msg.lat
        print(f"N:{current_n}, E:{current_e} ...")

        # 计算距离平方 (避免开根号运算，稍微快一点点，虽然Python并不在乎)
        dist_sq = (target_n - current_n) ** 2 + (target_e - current_e) ** 2

        # 实时打印距离，方便你看着爽
        print(f"距离目标还有: {math.sqrt(dist_sq):.2f} m")

        if dist_sq < (tolerance ** 2):
            print(">>> 已到达路点! <<<")
            break

        time.sleep(0.2)


# --- 1. 定义距离计算辅助函数 (优化版) ---
def calculate_gauss_distance(lon1, lat1, lon2, lat2):
    """
    计算两点间的平面投影距离 (米)
    为了性能，这里每次调用会重新计算投影中心，
    如果在极短距离内高频调用，建议把 transformer 做成全局缓存。
    """
    # 确定中央子午线 (3度带)
    central_meridian = round(lon1 / 3) * 3

    # 定义投影 (使用 EPSG:4326 作为源，自定义 tmerc 作为目标)
    proj_str = f"+proj=tmerc +lat_0=0 +lon_0={central_meridian} +k=1 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m"

    # 初始化转换器 (从 经纬度 -> 米)
    # always_xy=True 确保输入输出循序是 (lon, lat) -> (x, y)
    transformer = Transformer.from_crs("EPSG:4326", proj_str, always_xy=True)

    # 转换
    x1, y1 = transformer.transform(lon1, lat1)
    x2, y2 = transformer.transform(lon2, lat2)

    # 欧氏距离
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# --- 2. 阻塞等待到达函数 ---
def wait_until_arrived_2(target_lon, target_lat, target_alt, tolerance=1.5):
    """
    阻塞直到到达目标航点
    参数:
      target_lon: 目标经度 (如 113.xxx)
      target_lat: 目标纬度 (如 22.xxx)
      target_alt: 目标相对高度 (米)
      tolerance:  水平误差容限 (米)
    """
    print(f"✈️ [导航] 飞往: Lon {target_lon:.6f}, Lat {target_lat:.6f}, H {target_alt}m")

    while True:
        # 1. 安全地从缓存读取数据 (防止和后台转发线程冲突)
        if 'GLOBAL_POSITION_INT' not in master.messages:
            # 还没收到过 GPS 消息
            time.sleep(0.2)
            continue

        # 获取最新的一帧数据
        msg = master.messages['GLOBAL_POSITION_INT']

        # 2. 数据解码 (整数转浮点)
        curr_lon = msg.lon / 1e7
        curr_lat = msg.lat / 1e7
        curr_alt = msg.relative_alt / 1000.0  # 毫米 -> 米
        print(f"\n{curr_lon}\n{curr_lat}\n{curr_alt}")

        # 3.计算水平距离 (使用高斯投影)
        h_dist = calculate_gauss_distance(curr_lon, curr_lat, target_lon, target_lat)

        # 计算垂直距离 (直接相减)
        v_dist = abs(target_alt - curr_alt)

        # 打印状态 (每秒刷屏太快的话，可以加个计数器限制打印频率)
        print(f"📍 距目标 -> 水平: {h_dist:.2f}m | 垂直: {v_dist:.1f}m | 误差容限: {tolerance}m")

        # 4. 判断是否到达
        # 这里你可以决定是“只看水平距离”还是“水平+高度”都满足
        if h_dist < tolerance:
            # 如果对高度也有严格要求，解锁下面这行:
            # if h_dist < tolerance and v_dist < 1.0:
            print(f"✅ >>> 到达航点! (误差: {h_dist:.2f}m) <<<")
            break

        # 降低 CPU 占用，给转发线程留时间
        time.sleep(0.5)


# --- 关键函数: 切换模式 ---
# 由于V12在发送指令的同时会自动切换模式,所以不用单独切换模式
def change_mode(mode_name):
    # 获取飞控目前支持的模式ID
    mode_id = master.mode_mapping().get(mode_name)
    # print(f"guided模式id: {mode_id}")
    if mode_id is None:
        print(f"不支持的模式: {mode_name}")
        sys.exit(1)

    # print(f"正在切换到 {mode_name} 模式...")
    # 发送设置模式指令
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        18)

    # 循环等待，直到模式真的变过来
    while True:
        # 监听心跳包来确认模式
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        print(f"模式: {msg}")
        # 检查当前模式是否已变更
        if msg.custom_mode == mode_id:
            print(f"模式已切换为: {mode_name}")
            break
        time.sleep(0.1)


# --- 主程序：发送控制指令示例 ---
if __name__ == '__main__':
    # 启动接收线程
    # t = threading.Thread(target=connection_loop)
    # t.daemon = True
    # t.start()
    # --- 启动双线程 ---
    # 启动 飞控 -> 地面站 转发线程
    t_fc = threading.Thread(target=fc_to_gcs_loop)
    t_fc.daemon = True
    t_fc.start()

    # 启动 地面站 -> 飞控 转发线程 (新增)
    t_gcs = threading.Thread(target=gcs_to_fc_loop)
    t_gcs.daemon = True
    t_gcs.start()

    # 等待收到心跳
    master.wait_heartbeat()
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,  # 请求所有类型的数据(或者用 MAV_DATA_STREAM_POSITION)
        2,  # 请求频率 (Hz)，这里设为 4Hz (每秒4次) 足够了
        1  # 开启 (start)
    )
    # 1. 第一步：切模式
    # change_mode('GUIDED')

    # 2. 第二步：解锁
    # print("正在解锁...")
    # todo:解锁方式一
    # master.arducopter_arm()
    # master.motors_armed_wait()

    # todo:解锁方式二
    # master.mav.command_long_send(
    #     master.target_system,
    #     master.target_component,
    #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    #     0,
    #     1,  # Param 1: 1 表示解锁
    #     0,  # Param 2: 0 (不强制)
    #     0, 0, 0, 0, 0  # Param 3-7: 未使用
    # )
    #
    # # --- 2. 使用手动循环 替代 motors_armed_wait ---
    # # 既然封装函数卡住，我们就自己看心跳包里的 base_mode 字段
    # print("⏳ 正在等待解锁确认 (手动检查 Heartbeat)...")
    #
    # start_time = time.time()
    # while True:
    #     # 设定一个超时保护，防止死循环
    #     if time.time() - start_time > 10:
    #         print("❌ 解锁超时！飞控可能拒绝了解锁请求 (请检查是否有报错ACK)。")
    #         break
    #
    #     # 1. 优先检查 ACK (看看飞控拒没拒绝)
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=False)
    #     if ack_msg and ack_msg.command == 400:
    #         print(f"📩 收到解锁指令回执: Result = {ack_msg.result}")
    #         if ack_msg.result != 0:
    #             print(f"⚠️ 飞控拒绝解锁! 错误码: {ack_msg.result}")
    #             # 常见错误：1=暂时拒绝(正在初始化?), 4=失败(自检不过?)
    #
    #     # 2. 检查心跳包的状态位
    #     msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    #
    #     if msg:
    #         # MAV_MODE_FLAG_SAFETY_ARMED 的值是 128 (二进制 10000000)
    #         # 我们用 位与运算(&) 检查 base_mode 的第7位是否为 1
    #         is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    #         print(f"base mode:{msg.base_mode}")
    #
    #         if is_armed:
    #             print("🎉 检测到飞控已解锁 (Base_mode Check Passed)！")
    #             break
    #         else:
    #             # 还在锁定状态，继续等下一条心跳
    #             pass

    print(">>> 准备发送指令...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # 确认
        0, 0, 0, 0, 0, 0,  # 参数1-6 (通常不用填)
        10)  # 参数7: 高度
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            # 这里的 relative_alt 单位是毫米，所以要除以1000
            current_alt = msg.relative_alt / 1000.0
            print(f"当前高度: {current_alt:.2f} m")

            if current_alt >= 10 * 0.95:
                print("到达目标高度! 悬停中...")
                break

        time.sleep(0.5)

    # 到达后，你可以让它停在这，或者接你的 YOLO 逻辑
    # while True: ...
    # --- 扩充动作 1：水平移动 (使用 NED 坐标系) ---
    # NED 代表 North(北), East(东), Down(下)。
    # master.mav.set_position_target_local_ned_send(
    #     0,  # boot_time
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    #     0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    #     20, 20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    #     0, 0, 0,  # 速度 (忽略)
    #     0, 0, 0,  # 加速度 (忽略)
    #     0, 0)  # 偏航角 (忽略)

    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_NAV_LAND,
    #     0,  # 确认
    #     0, 0, 0, 0, 0, 0,  # 参数1-6 (通常不用填)
    #     0)  # 参数7: 高度
    # time.sleep(15)

    master.mav.command_int_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame: 使用本地NED坐标系
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # Command: 16 (导航到航点)
        0,  # current: 0 (不是任务列表中的点)
        0,  # autocontinue: 0
        50,  # param1: Hold time (悬停时间，秒，0表示不悬停)
        mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # param2: Accept Radius (接受半径，0表示使用默认)
        0,  # param3: Pass Radius (穿过半径，0表示使用默认)
        0,  # param4: Yaw (偏航角，NaN或0表示不改变朝向)
        int(376280473),  # X: 北向 20米 (必须转为int)
        int(-1223895053),  # Y: 东向 -20米 (即西向20米) (必须转为int)
        float(10)  # Z: 高度 -10米 (注意: command_int协议中 Z 字段是float类型!)
    )

    # time.sleep(10)  # 给无人机一点飞行时间
    wait_until_arrived_2(-122.3895053, 37.6280473, 10)
    time.sleep(1)
    print("\n🌱     第一点      \n")

    # master.mav.set_position_target_local_ned_send(
    #     0,  # boot_time
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    #     0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    #     20, -20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    #     0, 0, 0,  # 速度 (忽略)
    #     0, 0, 0,  # 加速度 (忽略)
    #     0, 0)  # 偏航角 (忽略)

    # master.mav.command_int_send(
    #     master.target_system,  # target_system
    #     master.target_component,  # target_component
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame: 使用本地NED坐标系
    #     mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # Command: 16 (导航到航点)
    #     0,  # current: 0 (不是任务列表中的点)
    #     0,  # autocontinue: 0
    #     50,  # param1: Hold time (悬停时间，秒，0表示不悬停)
    #     0,  # param2: Accept Radius (接受半径，0表示使用默认)
    #     0,  # param3: Pass Radius (穿过半径，0表示使用默认)
    #     0,  # param4: Yaw (偏航角，NaN或0表示不改变朝向)
    #     int(-1223892300),  # X: 北向 20米 (必须转为int)
    #     int(376278964),  # Y: 东向 -20米 (即西向20米) (必须转为int)
    #     float(10)  # Z: 高度 -10米 (注意: command_int协议中 Z 字段是float类型!)
    # )

    master.mav.command_int_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # Command: 16 (导航到航点)
        0,  # current: 0 (不是任务列表中的点)
        0,  # autocontinue: 0
        50,  # param1: Hold time (悬停时间，秒，0表示不悬停)
        mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # param2: Accept Radius (接受半径，0表示使用默认)
        0,  # param3: Pass Radius (穿过半径，0表示使用默认)
        0,  # param4: Yaw (偏航角，NaN或0表示不改变朝向)
        int(376277489),  # X: 北向 20米 (必须转为int)
        int(-1223897457),  # Y: 东向 -20米 (即西向20米) (必须转为int)
        float(10)  # Z: 高度 -10米 (注意: command_int协议中 Z 字段是float类型!)
    )
    wait_until_arrived_2(-122.3897457, 37.6277489, 10)
    time.sleep(0.5)
    print("\n🌱    第二点    \n")

    # master.mav.set_position_target_local_ned_send(
    #     0,  # boot_time
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    #     0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    #     -20, -20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    #     0, 0, 0,  # 速度 (忽略)
    #     0, 0, 0,  # 加速度 (忽略)
    #     0, 0)  # 偏航角 (忽略)
    master.mav.command_int_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame: 使用本地NED坐标系
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # Command: 16 (导航到航点)
        0,  # current: 0 (不是任务列表中的点)
        0,  # autocontinue: 0
        50,  # param1: Hold time (悬停时间，秒，0表示不悬停)
        mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # param2: Accept Radius (接受半径，0表示使用默认)
        0,  # param3: Pass Radius (穿过半径，0表示使用默认)
        0,  # param4: Yaw (偏航角，NaN或0表示不改变朝向)
        int(376277595),  # X: 北向 20米 (必须转为int)
        int(-1223893317),  # Y: 东向 -20米 (即西向20米) (必须转为int)
        float(10)  # Z: 高度 -10米 (注意: command_int协议中 Z 字段是float类型!)
    )
    wait_until_arrived_2(-122.3893317, 37.6277595, 10)
    time.sleep(0.5)
    print("\n🌱     第三点      \n")

    # master.mav.set_position_target_local_ned_send(
    #     0,  # boot_time
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    #     0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    #     -20, 20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    #     0, 0, 0,  # 速度 (忽略)
    #     0, 0, 0,  # 加速度 (忽略)
    #     0, 0)  # 偏航角 (忽略)
    # master.mav.command_int_send(
    #     master.target_system,  # target_system
    #     master.target_component,  # target_component
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame: 使用本地NED坐标系
    #     mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # Command: 16 (导航到航点)
    #     0,  # current: 0 (不是任务列表中的点)
    #     0,  # autocontinue: 0
    #     50,  # param1: Hold time (悬停时间，秒，0表示不悬停)
    #     mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # param2: Accept Radius (接受半径，0表示使用默认)
    #     0,  # param3: Pass Radius (穿过半径，0表示使用默认)
    #     0,  # param4: Yaw (偏航角，NaN或0表示不改变朝向)
    #     int(376278574),  # X: 北向 20米 (必须转为int)
    #     int(-1223895156),  # Y: 东向 -20米 (即西向20米) (必须转为int)
    #     float(20)  # Z: 高度 -10米 (注意: command_int协议中 Z 字段是float类型!)
    # )
    # wait_until_arrived_2(-122.3895156, 37.6278574, 20)
    # time.sleep(0.5)
    # print("\n🌱     第四点      \n")

    # master.mav.set_position_target_local_ned_send(
    #     0,  # boot_time
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    #     0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    #     0, 0, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    #     0, 0, 0,  # 速度 (忽略)
    #     0, 0, 0,  # 加速度 (忽略)
    #     0, 0)  # 偏航角 (忽略)
    # todo:返航有没有打包好的指令?
    # master.mav.command_int_send(
    #     master.target_system,  # target_system
    #     master.target_component,  # target_component
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame: 使用本地NED坐标系
    #     mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # Command: 16 (导航到航点)
    #     0,  # current: 0 (不是任务列表中的点)
    #     0,  # autocontinue: 0
    #     50,  # param1: Hold time (悬停时间，秒，0表示不悬停)
    #     mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # param2: Accept Radius (接受半径，0表示使用默认)
    #     0,  # param3: Pass Radius (穿过半径，0表示使用默认)
    #     0,  # param4: Yaw (偏航角，NaN或0表示不改变朝向)
    #     int(0),  # X: 北向 20米 (必须转为int)
    #     int(0),  # Y: 东向 -20米 (即西向20米) (必须转为int)
    #     float(-10)  # Z: 高度 -10米 (注意: command_int协议中 Z 字段是float类型!)
    # )
    # wait_until_arrived(0, 0)

    # --- 扩充动作 2：原地掉头 (改变航向 Yaw) ---
    # 改变无人机的朝向，比如转到 180度（南）
    # print("动作：原地旋转，面向正南 (180度)")
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    #     0,
    #     180,  # 目标角度 (0-360)
    #     20,  # 旋转速度 (度/秒)
    #     1,  # 方向: 1=顺时针, -1=逆时针
    #     0,  # 0=绝对角度, 1=相对角度
    #     0, 0, 0)
    # --- 扩充动作 3：自动降落 (Land) ---
    # print("动作：开始自动降落")
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_NAV_LAND,
    #     0, 0, 0, 0, 0, 0, 0, 0)

    # todo:返航指令
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0)

    time.sleep(2)
    # 监控高度直到着陆
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt_m = msg.relative_alt / 1000.0
        print(f"降落中... 当前高度: {alt_m:.2f} 米")
        if alt_m < 0.3:
            print("已着陆.")
            break

    # 示例：每隔 5 秒向飞控请求一次参数，或者发送解锁指令
    # 这里我们只是简单的发心跳维持连接
    # while True:
    #     # 很多飞控需要你发心跳给它，它才理你
    #     master.mav.heartbeat_send(
    #         mavutil.mavlink.MAV_TYPE_GCS,
    #         mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    #         0, 0, 0
    #     )
    #     time.sleep(1)
