import paho.mqtt.client as mqtt
import json
import time

# --- 这里填工作人员给你的信息 ---
BROKER_ADDRESS = "192.168.1.100"  # 或者是 fly.vkfly.com 这种域名
PORT = 1883
TOPIC_PUB = "vkv12/control/mode"  # 发松指令的主题(举例)
TOPIC_SUB = "vkv12/status"  # 接收状态的主题(举例)


# 1. 定义连接成功后的回调
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ MQTT 连接成功!")
        # 订阅状态，看看飞控回什么
        client.subscribe(TOPIC_SUB)
    else:
        print(f"❌ 连接失败, 错误码: {rc}")


# 2. 定义收到消息的回调
def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        print(f"📩 收到来自 {msg.topic} 的消息: {payload}")
    except:
        print(f"📩 收到无法解析的消息")


# 3. 初始化客户端
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# 4. 开始连接
print("⏳ 正在连接 MQTT Broker...")
try:
    client.connect(BROKER_ADDRESS, PORT, 60)
    # 启动后台线程处理网络流量
    client.loop_start()
except Exception as e:
    print(f"连接报错: {e}")
    exit()

# 5. 发送指令切换模式！
# 注意：这里的 JSON 格式必须也就是那个 "SDK API Documentation" 里写的格式
# 我这里是猜的，你需要看文档填入正确的 Key-Value
command_payload = {
    "command": "set_mode",
    "mode_id": 18,
    "mode_name": "GUIDED"
}

# 把字典转换成 JSON 字符串
json_str = json.dumps(command_payload)

print(f"🚀 向主题 {TOPIC_PUB} 发送指令: {json_str}")
client.publish(TOPIC_PUB, json_str)

# 保持运行一会看看有没有回复
time.sleep(5)
client.loop_stop()
