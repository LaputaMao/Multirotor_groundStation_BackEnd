from pymavlink import mavutil
import time

# 请替换为你的实际端口
FC_PORT = 'COM6'
BAUD_RATE = 57600

print(f"尝试连接飞控 {FC_PORT}...")
master = mavutil.mavlink_connection(FC_PORT, baud=BAUD_RATE)
master.wait_heartbeat()
print(f"✅ 连接成功! Autopilot ID: {master.field('HEARTBEAT', 'autopilot', 0)}")
print(f"当前模式 ID: {master.field('HEARTBEAT', 'custom_mode', 0)}")

# 定义我们要切换的目标模式 ID (VKFLY_CUSTOM_MODE_GUIDE)
TARGET_MODE_ID = 27

print(f"\n⚡ 正在尝试强行切换到模式 {TARGET_MODE_ID} (GUIDE)...")

# --- 核心修改：使用 command_long_send ---
# 指令: MAV_CMD_DO_SET_MODE (176)
# 参数1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
# 参数2: 你的目标模式 ID (18)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command 176
    0,  # confirmation
    1,  # param1: 开启 Custom Mode (必填1)
    TARGET_MODE_ID,  # param2: 目标模式 ID
    0, 0, 0, 0, 0  # param3-7: 未使用
)
# todo 切换模式要使用 command_long_send set_mode_send方法不可用
# master.mav.set_mode_send(
#     master.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     18)

# --- 关键步骤：监听 ACK (回执) ---
print(" 等待飞控回复 (ACK)...")
while True:
    # 接收 COMMAND_ACK 消息
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

    if msg is None:
        print("❌ 切换超时！飞控没有回复 ACK。可能是连接问题。")
        break

    # 检查是不是回复我们要的那个指令 (176)
    if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        print(f" 收到回执: result = {msg.result}")

        if msg.result == 0:  # MAV_RESULT_ACCEPTED
            print("🎉 切换成功！飞控接受了指令！")
        elif msg.result == 1:  # MAV_RESULT_TEMPORARILY_REJECTED
            print("⛔ 暂时拒绝！可能是前置条件不满足 (比如还没解锁？还是没GPS？)")
        elif msg.result == 2:  # MAV_RESULT_DENIED
            print(" 拒绝执行！飞控不支持这个指令，或者模式ID不对。")
        elif msg.result == 3:  # MAV_RESULT_UNSUPPORTED
            print("❓ 不支持！飞控不懂什么是 DO_SET_MODE。")
        else:
            print(f"⚠️ 其他错误代码: {msg.result}")
        break

# 再读一次心跳确认一下
msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(f"🔄 最终确认当前模式 ID: {msg.custom_mode}")
