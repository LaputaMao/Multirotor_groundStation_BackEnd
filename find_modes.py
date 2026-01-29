from pymavlink import mavutil
import time

# 这里填你的串口，如果是 Windows 直连
FC_PORT = 'COM6'
BAUD_RATE = 57600

print(f"🕵️ 正在连接 {FC_PORT} 进行侦听...")
try:
    master = mavutil.mavlink_connection(FC_PORT, baud=BAUD_RATE)
except:
    print("❌ 连接失败，请检查端口或关闭其他占用的软件")
    exit()

print("✅ 连接成功！")
print("👉 尝试切换不同的模式（比如‘姿态’、‘定点’、‘程控/引导’）")
print("⬇️ 下面会显示飞控实际所处的模式 ID 和名称：")
print("-" * 40)

last_mode = None

while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        18)
    if msg:
        # 获取模式的数字 ID (base_mode) 和 自定义模式 ID (custom_mode)
        # 大多数飞控是通过 custom_mode 来区分的
        current_custom_mode = msg.custom_mode
        current_base_mode = msg.base_mode

        # 尝试让 pymavlink 帮我们翻译名字
        try:
            mode_name = mavutil.mode_string_v10(msg)
        except:
            mode_name = "Unknown"

        # 只有模式变化时才打印，防止刷屏
        if current_custom_mode != last_mode:
            msg = master.recv_match(type='HEARTBEAT', blocking=True)
            print(f"模式: {msg}")
            print(
                f"🔁 模式切换检测! \n   名称: [{mode_name}] \n   Custom ID: {current_custom_mode} \n   Base ID: {current_base_mode}")
            print("-" * 40)
            last_mode = current_custom_mode
