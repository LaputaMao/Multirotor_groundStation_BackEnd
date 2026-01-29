from pymavlink import mavutil
import time
import sys

# 连接字符串 (SITL)
# connection_string = 'udp:127.0.0.1:5353'
# print(f"正在连接: {connection_string}")
# master = mavutil.mavlink_connection(connection_string)
# master = mavutil.mavlink_connection('COM6', baud=115200)
# 这里填你的串口，如果是 Windows 直连
FC_PORT = 'COM6'
BAUD_RATE = 57600

print(f" 正在连接 {FC_PORT} 进行侦听...")
try:
    master = mavutil.mavlink_connection(FC_PORT, baud=BAUD_RATE)
except:
    print("❌ 连接失败，请检查端口或关闭其他占用的软件")
    exit()

print("✅ 连接成功！")
print("-" * 40)

master.wait_heartbeat()
# --- 新增步骤：请求数据流 ---
print("正在请求数据流...")
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,  # 请求所有类型的数据(或者用 MAV_DATA_STREAM_POSITION)
    1,  # 请求频率 (Hz)，这里设为 4Hz (每秒4次) 足够了
    1  # 开启 (start)
)
msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(f"模式: {msg}")
print("连接成功!")


# --- 关键函数: 切换模式 ---
def change_mode(mode_name):
    # 获取飞控目前支持的模式ID
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"不支持的模式: {mode_name}")
        sys.exit(1)

    print(f"正在切换到 {mode_name} 模式...")
    # 发送设置模式指令
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    # 循环等待，直到模式真的变过来
    while True:
        # 监听心跳包来确认模式
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        # 检查当前模式是否已变更
        if msg.custom_mode == mode_id:
            print(f"模式已切换为: {mode_name}")
            break
        time.sleep(0.1)


# NED 坐标系距离检查函数
def wait_until_arrived(target_n, target_e, tolerance=0.5):
    """
    阻塞直到到达目标点
    tolerance: 容差范围，比如 0.5 米
    """
    print(f"正在飞往 N:{target_n}, E:{target_e} ...")
    while True:
        # 获取当前位置 (你需要订阅 LOCAL_POSITION_NED 消息)
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_n = msg.x
        current_e = msg.y

        # 计算距离平方 (避免开根号运算，稍微快一点点，虽然Python并不在乎)
        dist_sq = (target_n - current_n) ** 2 + (target_e - current_e) ** 2

        # 实时打印距离，方便你看着爽
        # print(f"距离目标还有: {math.sqrt(dist_sq):.2f} m")

        if dist_sq < (tolerance ** 2):
            print(">>> 已到达路点! <<<")
            break

        time.sleep(0.2)


# --- 主流程 ---

# 1. 第一步：切模式
# change_mode('GUIDED')

# 2. 第二步：解锁
print("正在解锁...")
# master.arducopter_arm()
# master.motors_armed_wait()
# --- 1. 使用 command_long_send 替代 arducopter_arm ---
# 指令: MAV_CMD_COMPONENT_ARM_DISARM (400)
# 参数1: 1 = 解锁 (ARM), 0 = 上锁 (DISARM)
# 参数2: 0 = 强制标识 (通常填0，如果飞控报"Arming check fail"且你想强制飞，可填 21196，但极不推荐)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # Param 1: 1 表示解锁
    0,  # Param 2: 0 (不强制)
    0, 0, 0, 0, 0  # Param 3-7: 未使用
)

# --- 2. 使用手动循环 替代 motors_armed_wait ---
# 既然封装函数卡住，我们就自己看心跳包里的 base_mode 字段
print("⏳ 正在等待解锁确认 (手动检查 Heartbeat)...")

start_time = time.time()
while True:
    # 设定一个超时保护，防止死循环
    if time.time() - start_time > 10:
        print("❌ 解锁超时！飞控可能拒绝了解锁请求 (请检查是否有报错ACK)。")
        break

    # 1. 优先检查 ACK (看看飞控拒没拒绝)
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=False)
    if ack_msg and ack_msg.command == 400:
        print(f"📩 收到解锁指令回执: Result = {ack_msg.result}")
        if ack_msg.result != 0:
            print(f"⚠️ 飞控拒绝解锁! 错误码: {ack_msg.result}")
            # 常见错误：1=暂时拒绝(正在初始化?), 4=失败(自检不过?)

    # 2. 检查心跳包的状态位
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

    if msg:
        # MAV_MODE_FLAG_SAFETY_ARMED 的值是 128 (二进制 10000000)
        # 我们用 位与运算(&) 检查 base_mode 的第7位是否为 1
        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        print(msg.base_mode)

        if is_armed:
            print("🎉 检测到飞控已解锁 (Base_mode Check Passed)！")
            break
        else:
            # 还在锁定状态，继续等下一条心跳
            pass

# 3. 第三步：起飞
target_altitude = 10  # 米
print(f"发送起飞指令 -> {target_altitude}m")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,  # 确认
    0, 0, 0, 0, 0, 0,  # 参数1-6 (通常不用填)
    target_altitude)  # 参数7: 高度
# 4. 第四步：监控起飞状态
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        # 这里的 relative_alt 单位是毫米，所以要除以1000
        current_alt = msg.relative_alt / 1000.0
        print(f"当前高度: {current_alt:.2f} m")

        if current_alt >= target_altitude * 0.95:
            print("到达目标高度! 悬停中...")
            break

    time.sleep(0.5)

# 到达后，你可以让它停在这，或者接你的 YOLO 逻辑
# while True: ...
# --- 扩充动作 1：水平移动 (使用 NED 坐标系) ---
# NED 代表 North(北), East(东), Down(下)。
master.mav.set_position_target_local_ned_send(
    0,  # boot_time
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    20, 20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    0, 0, 0,  # 速度 (忽略)
    0, 0, 0,  # 加速度 (忽略)
    0, 0)  # 偏航角 (忽略)

# time.sleep(10)  # 给无人机一点飞行时间
wait_until_arrived(20, 20)
time.sleep(1)
print("\n🌱     播种中...      \n")

master.mav.set_position_target_local_ned_send(
    0,  # boot_time
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    20, -20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    0, 0, 0,  # 速度 (忽略)
    0, 0, 0,  # 加速度 (忽略)
    0, 0)  # 偏航角 (忽略)
wait_until_arrived(20, -20)
time.sleep(0.5)
print("\n🌱     播种中...      \n")

master.mav.set_position_target_local_ned_send(
    0,  # boot_time
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    -20, -20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    0, 0, 0,  # 速度 (忽略)
    0, 0, 0,  # 加速度 (忽略)
    0, 0)  # 偏航角 (忽略)
wait_until_arrived(-20, -20)
time.sleep(0.5)
print("\n🌱     播种中...      \n")

master.mav.set_position_target_local_ned_send(
    0,  # boot_time
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    -20, 20, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    0, 0, 0,  # 速度 (忽略)
    0, 0, 0,  # 加速度 (忽略)
    0, 0)  # 偏航角 (忽略)
wait_until_arrived(-20, 20)
time.sleep(0.5)
print("\n🌱     播种中...      \n")

master.mav.set_position_target_local_ned_send(
    0,  # boot_time
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
    0b0000111111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
    0, 0, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
    0, 0, 0,  # 速度 (忽略)
    0, 0, 0,  # 加速度 (忽略)
    0, 0)  # 偏航角 (忽略)
wait_until_arrived(0, 0)
# --- 扩充动作 3：自动降落 (Land) ---
print("动作：开始自动降落")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0)
# --- 扩充动作 2：原地掉头 (改变航向 Yaw) ---
# 改变无人机的朝向，比如转到 180度（南）
print("动作：原地旋转，面向正南 (180度)")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0,
    180,  # 目标角度 (0-360)
    20,  # 旋转速度 (度/秒)
    1,  # 方向: 1=顺时针, -1=逆时针
    0,  # 0=绝对角度, 1=相对角度
    0, 0, 0)

time.sleep(2)
# 监控高度直到着陆
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt_m = msg.relative_alt / 1000.0
    print(f"降落中... 当前高度: {alt_m:.2f} 米")
    if alt_m < 0.3:
        print("已着陆，电机锁定。")
        break

print("\n-播种任务结束-")
