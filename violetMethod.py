from pymavlink import mavutil
import time
import threading

# --- 配置 ---
FC_PORT = 'COM6'  # 你的端口
BAUD_RATE = 57600
TARGET_MODE_ID = 18  # VKFLY_CUSTOM_MODE_GUIDE

# 连接
try:
    master = mavutil.mavlink_connection(FC_PORT, baud=BAUD_RATE)
    print("等待心跳...")
    master.wait_heartbeat()
    print("✅ 连接成功")
except Exception as e:
    print(e)
    exit()


# --- 1. 启动一个线程，疯狂发送“位置设定点” ---
# 很多飞控如果不收到这个数据包，就会拒绝进入 GUIDE 模式
def send_position_target():
    while True:
        # 发送 SET_POSITION_TARGET_LOCAL_NED
        # 告诉飞控：保持当前位置 (0,0,0) 速度为0
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            # type_mask: 忽略加速度/偏航率，只控制位置+速度 (二进制掩码)
            0b110111111000,
            0, 0, 0,  # x, y, z (位置 0)
            0, 0, 0,  # vx, vy, vz (速度 0)
            0, 0, 0,  # ax, ay, az
            0, 0  # yaw, yaw_rate
        )
        time.sleep(0.2)  # 5Hz 频率


# 启动发送线程 (作为后台守护线程)
t = threading.Thread(target=send_position_target)
t.daemon = True
t.start()
print("📡 已启动位置数据流发送 (欺骗飞控已准备好导航)...")
time.sleep(2)  # 让飞控先吃几口数据

# --- 2. 尝试解锁 (ARM) ---
# 注意：如果你是在室内测试，请确保没桨！或者你可以注释掉这段，手动用遥控器解锁
print("🔓 正在尝试解锁 (ARM)...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0  # param1=1 表示解锁
)
time.sleep(2)  # 给它点时间反应

# --- 3. 正式切换模式 ---
print(f"🚀 正在切换到模式 {TARGET_MODE_ID} ...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    1,  # Model Flag: Custom Mode
    TARGET_MODE_ID,  # 目标ID 18
    0, 0, 0, 0, 0
)

# 监听回执
while True:
    msg = master.recv_match(type=['COMMAND_ACK', 'HEARTBEAT'], blocking=True)

    if msg.get_type() == 'COMMAND_ACK' and msg.command == 176:
        print(f"📩 切换模式回执 Result: {msg.result}")
        if msg.result == 0:
            print("🎉 成功进入指点飞行模式！")
            break
        elif msg.result == 2:
            print("❌ 依然被拒绝 (Result 2)。可能需要 GPS 3D Fix 锁定？")
            break

    # 也可以看心跳变没变
    if msg.get_type() == 'HEARTBEAT':
        if msg.custom_mode == TARGET_MODE_ID:
            print(f"🎉 心跳包确认：已处于模式 {msg.custom_mode}！")
            break
