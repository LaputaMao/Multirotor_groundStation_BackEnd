import sys
import threading

import numpy as np
import rasterio
from rasterio.features import shapes
from shapely.geometry import shape, LineString, Polygon, box
from pymavlink import mavutil
import time
from collections import deque
from scipy.ndimage import uniform_filter
from pyproj import Proj, Transformer
import random
import math
from shapely.geometry import box, Polygon
import socket


# ==========================================
# 模块1: DEM 分析与坡度计算
# ==========================================
class DemAnalyzer:
    def __init__(self, tiff_path):
        # 使用 masked=True 读取，Rasterio 会自动把 NoData 标记出来
        # 这是解决 "overflow" 的关键！
        with rasterio.open(tiff_path) as src:
            self.src = src
            # 读取第一层数据，作为 Masked Array
            masked_elev = src.read(1, masked=True)
            self.transform = src.transform
            self.res_x = self.transform[0]
            self.res_y = -self.transform[4]

            # --- 数据清洗 ---
            # 1. 提取有效数据的平均值
            mean_val = masked_elev.mean()

            print(f"地图平均海拔: {mean_val:.2f} 米")

            # 2. 填充无效值
            # 这里的技巧是：把无效区域(NoData)填充为平均海拔。
            # 这样在计算 TPI 和 坡度 时，边缘不会出现剧烈的“悬崖”，
            # 从而避免生成错误的 steep slope 或 basin。
            self.elevation = masked_elev.filled(mean_val)

    def detect_suitable_seeding_zones(self, max_slope_deg=25, tpi_window_size=9):
        print("正在进行地形识别 (坡度 + 凹陷分析)...")

        # --- 步骤 1: 计算坡度 (Slope) ---
        # 使用 numpy.gradient 计算梯度
        dy, dx = np.gradient(self.elevation, self.res_y, self.res_x)

        # 处理可能的除0错误或非法值
        with np.errstate(divide='ignore', invalid='ignore'):
            slope_rad = np.arctan(np.sqrt(dx ** 2 + dy ** 2))
            slope_deg = np.degrees(slope_rad)

        # 将 NaN 的坡度设为 90度 (不可飞)
        slope_deg = np.nan_to_num(slope_deg, nan=90.0)

        # 掩膜A: 坡度合格
        slope_mask = (slope_deg <= max_slope_deg)

        # --- 步骤 2: 计算 TPI (地形位置指数) ---
        # uniform_filter 计算局部平均
        local_mean = uniform_filter(self.elevation, size=tpi_window_size, mode='reflect')
        tpi = self.elevation - local_mean

        # 掩膜B: 必须是凹陷/盆地地形
        # TPI < -0.15: 稍微加深一点阈值，过滤掉平地上的微小噪点
        basin_mask = (tpi < -0.15)

        # --- 步骤 3: 取交集 ---
        final_mask = np.logical_or(slope_mask, basin_mask).astype('uint8')

        # --- 步骤 4: 转为多边形 ---
        valid_polygons = []
        for geom, val in shapes(final_mask, transform=self.transform):
            if val == 1:
                s = shape(geom)
                # 过滤掉太小的区域(比如 < 10m^2) 和 太奇怪的狭长区域
                if s.area > 10:
                    # 可选：可以用 s.simplify(0.5) 简化一下形状，减少航点数
                    valid_polygons.append(s)

        print(f"识别完成：找到 {len(valid_polygons)} 个符合条件的播种区。")
        return valid_polygons


class SmartPathPlanner:
    def __init__(self, swath_width=5.0):
        self.swath_width = swath_width  # 这里的单位依然保持是：米
        # 纬度 1度 ≈ 111132 米
        self.METERS_PER_DEG_LAT = 111132

    def _estimate_distance(self, p1, p2):
        """
        辅助方法：估算两点经纬度的物理距离(米)
        用于贪心算法判断谁最近
        """
        lon1, lat1 = p1
        lon2, lat2 = p2

        # 简易平地近似公式
        avg_lat = (lat1 + lat2) / 2
        # 经度差转米
        dx = (lon1 - lon2) * (self.METERS_PER_DEG_LAT * math.cos(math.radians(avg_lat)))
        # 纬度差转米
        dy = (lat1 - lat2) * self.METERS_PER_DEG_LAT

        return math.hypot(dx, dy)

    def generate_snake_path(self, polygon):
        """
        全覆盖路径算法 (适配经纬度版)
        """
        minx, miny, maxx, maxy = polygon.bounds
        path_points = []

        # --- 关键修改 1: 将切条宽度(米) 转换为 纬度(度) ---
        # 每次 y 增加的步长不再是 5.0，而是 5.0 / 111132
        swath_step_deg = self.swath_width / self.METERS_PER_DEG_LAT

        # 1. 确定所有的扫描线 Y 坐标 (纬度)
        scan_ys = []
        # 从底部开始，留半个宽度的边距
        y = miny + (swath_step_deg / 2)
        while y < maxy:
            scan_ys.append(y)
            y += swath_step_deg

        # 2. 遍历每一行
        for i, current_y in enumerate(scan_ys):
            # 构造水平扫描线 (纬度线)
            # 为了保证能切到多边形，X轴范围稍微给大一点点无所谓
            line = LineString([(minx - 0.001, current_y), (maxx + 0.001, current_y)])
            intersection = line.intersection(polygon)

            if intersection.is_empty:
                continue

            # 3. 标准化线段 (处理凹多边形或复杂地形)
            if intersection.geom_type == 'MultiLineString':
                segs = list(intersection.geoms)
            else:
                segs = [intersection]

            # 4. 按经度(X)从小到大排序
            segs.sort(key=lambda s: s.bounds[0])

            base_coords = []
            for seg in segs:
                seg_coords = list(seg.coords)
                # 确保每一小段也是从西向东(经度增加方向)
                if seg_coords[0][0] > seg_coords[-1][0]:
                    seg_coords.reverse()
                base_coords.extend(seg_coords)

            # 5. 蛇形翻转：奇数行反转 (东 -> 西)
            if i % 2 == 1:
                base_coords.reverse()

            path_points.extend(base_coords)

        return path_points

    def sort_regions_greedy(self, start_pos, polygons):
        """
        贪心算法排序：决定先飞哪个区域 (适配经纬度版)
        """
        # start_pos 格式应该是 (lon, lat)
        current_pos = start_pos
        remaining_polys = polygons.copy()
        final_queue = deque()

        print("🧠 [规划] 正在计算全局飞行顺序 (贪心策略)...")

        while remaining_polys:
            best_dist = float('inf')
            best_poly_idx = -1
            best_path = []

            # 寻找离当前位置最近的区域起点
            for i, poly in enumerate(remaining_polys):
                # 生成该区域的蛇形路径
                path = self.generate_snake_path(poly)
                if not path: continue

                # 路径的第一个点就是该区域的起点
                start_node = path[0]

                # --- 关键修改 2: 使用经纬度转米的距离估算 ---
                dist = self._estimate_distance(current_pos, start_node)

                if dist < best_dist:
                    best_dist = dist
                    best_poly_idx = i
                    best_path = path

            # 选中了最近的区域
            if best_poly_idx != -1:
                # print(f"  👉 选中区域 {best_poly_idx+1}, 距离上一点 {best_dist:.1f}米")

                # 将该区域的所有点加入总队列
                for p in best_path:
                    # 可以在这里补充高度，凑成 (lon, lat, alt)
                    # 假设作业高度在外面统一控制，这里先存经纬度
                    final_queue.append(p)

                # 更新当前位置为该区域的最后一个点
                current_pos = best_path[-1]
                remaining_polys.pop(best_poly_idx)
            else:
                # 剩下的区域都无法生成路径(可能太小了)，跳过
                break

        print(f"✅ [规划] 完成! 总航点数: {len(final_queue)}")
        return final_queue


# ==========================================
# 模块2: 路径规划 (切条 + 贪心算法)
# ==========================================
# class SmartPathPlanner:
#     def __init__(self, swath_width=5.0):
#         self.swath_width = swath_width  # 航道宽度 5米
#
#     def generate_snake_path(self, polygon):
#         """
#         全覆盖路径算法 (修正版)：严格的蛇形 (Boustrophedon)
#         偶数行：左 -> 右
#         奇数行：右 -> 左
#         """
#         minx, miny, maxx, maxy = polygon.bounds
#         path_points = []
#
#         # 1. 确定所有的扫描线 Y 坐标
#         scan_ys = []
#         y = miny + (self.swath_width / 2)
#         while y < maxy:
#             scan_ys.append(y)
#             y += self.swath_width
#
#         # 2. 遍历每一行
#         for i, current_y in enumerate(scan_ys):
#             # 构造水平扫描线
#             line = LineString([(minx, current_y), (maxx, current_y)])
#             intersection = line.intersection(polygon)
#
#             if intersection.is_empty:
#                 continue
#
#             # 3. 标准化线段 (处理可能出现的多段线，比如 U 型地形)
#             if intersection.geom_type == 'MultiLineString':
#                 segs = list(intersection.geoms)
#             else:
#                 segs = [intersection]
#
#             # --- 关键步骤 A: 强制按 X 轴从小到大排序 ---
#             # 无论 shapely 怎么返回，我们先把线段按“从西向东”排好序
#             # 这样 base_coords 永远是：[(x_min, y), (x_mid, y)... (x_max, y)]
#             segs.sort(key=lambda s: s.bounds[0])
#
#             base_coords = []
#             for seg in segs:
#                 # 确保每一小段也是无脑从左到右
#                 # (shapely 的 coords 只有两个点，但也得防一手)
#                 seg_coords = list(seg.coords)
#                 if seg_coords[0][0] > seg_coords[-1][0]:
#                     seg_coords.reverse()
#                 base_coords.extend(seg_coords)
#
#             # --- 关键步骤 B: 根据行号(i) 决定是否翻转整个列表 ---
#             # 偶数行 (0, 2, 4...) : 保持 (左 -> 右)
#             # 奇数行 (1, 3, 5...) : 翻转 (右 -> 左)
#             if i % 2 == 1:
#                 base_coords.reverse()
#
#             path_points.extend(base_coords)
#
#         return path_points
#
#     def sort_regions_greedy(self, start_pos, polygons):
#         """
#         贪心算法排序：决定先飞哪个区域
#         """
#         current_pos = start_pos
#         remaining_polys = polygons.copy()
#         final_queue = deque()  # 最终的坐标队列
#
#         print("正在规划全局飞行顺序...")
#
#         while remaining_polys:
#             best_dist = float('inf')
#             best_poly_idx = -1
#             best_path = []
#
#             # 寻找离当前位置最近的区域起点
#             for i, poly in enumerate(remaining_polys):
#                 # 生成该区域的蛇形路径
#                 path = self.generate_snake_path(poly)
#                 if not path: continue
#
#                 # 路径的第一个点就是该区域的起点
#                 start_node = path[0]
#                 dist = math.hypot(start_node[0] - current_pos[0], start_node[1] - current_pos[1])
#
#                 if dist < best_dist:
#                     best_dist = dist
#                     best_poly_idx = i
#                     best_path = path
#
#             # 选中了最近的区域
#             if best_poly_idx != -1:
#                 # 将该区域的所有点加入总队列
#                 for p in best_path:
#                     final_queue.append(p)
#
#                 # 更新当前位置为该区域的最后一个点
#                 current_pos = best_path[-1]
#                 # 移除该区域
#                 remaining_polys.pop(best_poly_idx)
#             else:
#                 break  # 应该不会发生，除非所有区域都生成路径失败
#
#         return final_queue


# class DroneController:
#     def __init__(self, connection_str):
#         print(f"连接飞控: {connection_str}")
#         # self.master = mavutil.mavlink_connection('COM7', 57600, autoreconnect=True)
#         self.master = mavutil.mavlink_connection(connection_str)
#         self.master.wait_heartbeat()
#         print("正在请求数据流...")
#         self.master.mav.request_data_stream_send(
#             self.master.target_system,
#             self.master.target_component,
#             mavutil.mavlink.MAV_DATA_STREAM_ALL,  # 请求所有类型的数据(或者用 MAV_DATA_STREAM_POSITION)
#             1,  # 请求频率 (Hz)，这里设为 4Hz (每秒4次) 足够了
#             1  # 开启 (start)
#         )
#
#         print("飞控已连接!")
#         self.current_pos = (0, 0, 0)  # N, E, D
#
#     def change_mode(self, mode_name):
#         # 获取飞控目前支持的模式ID
#         mode_id = self.master.mode_mapping().get(mode_name)
#         if mode_id is None:
#             print(f"不支持的模式: {mode_name}")
#             sys.exit(1)
#
#         print(f"正在切换到 {mode_name} 模式...")
#         # 发送设置模式指令
#         self.master.mav.set_mode_send(
#             self.master.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_id)
#
#         # 循环等待，直到模式真的变过来
#         while True:
#             # 监听心跳包来确认模式
#             msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
#             # 检查当前模式是否已变更
#             if msg.custom_mode == mode_id:
#                 print(f"模式已切换为: {mode_name}")
#                 break
#             time.sleep(0.1)
#
#     def takeoff(self, alt=10):
#         # 起飞代码
#         print("正在解锁...")
#         self.master.arducopter_arm()
#         self.master.motors_armed_wait()
#         print("已解锁!")
#         print(f"起飞至 {alt}米...")
#         self.master.mav.command_long_send(
#             self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
#         # time.sleep(5)  # 简单等待起飞
#         # 监控起飞状态
#         while True:
#             msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#             if msg:
#                 # 这里的 relative_alt 单位是毫米，所以要除以1000
#                 current_alt = msg.relative_alt / 1000.0
#                 print(f"当前高度: {current_alt:.2f} m")
#
#                 if current_alt >= alt * 0.95:
#                     print("到达目标高度! 悬停中...")
#                     break
#
#             time.sleep(0.5)
#
#     def fly_mission(self, waypoint_queue):
#         """
#         核心执行逻辑：从队列取点 -> 飞 -> 等 -> 下一个
#         """
#         print(f"开始执行任务，共有 {len(waypoint_queue)} 个航点")
#
#         while len(waypoint_queue) > 0:
#             # 1. 从队列取出下一个点 (FIFO)
#             target = waypoint_queue.popleft()  # (x, y) 假设是投影坐标或NED
#
#             # 注意：如果DEM坐标是投影坐标(米)，你需要根据起飞点转换成相对坐标(NED)
#             # 这里假设 target 已经是相对于起飞点的 NED 坐标 (North, East)
#             target_n, target_e = target
#
#             print(f">>> 前往航点: N {target_n:.1f}, E {target_e:.1f}")
#
#             # 2. 发送指令
#             self.send_position_target(target_n, target_e, -10)  # 保持高度10米
#
#             # 3. 阻塞等待到达
#             self.wait_until_arrived(target_n, target_e)
#
#             # 4. 到达后执行动作，比如播种机开
#             self.check_and_sow()
#
#         print("所有航点执行完毕!")
#
#     def send_position_target(self, n, e, d):
#         # ... 你之前的 set_position_target_local_ned_encode 代码 ...
#         # 注意 type_mask 要忽略速度和加速度，只控制位置
#         self.master.mav.set_position_target_local_ned_send(
#             0, self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#             0b110111111000,  # 使用位置控制
#             n, e, d,
#             0, 0, 0, 0, 0, 0, 0, 0)
#
#     # NED 坐标系距离检查函数
#     def wait_until_arrived(self, target_n, target_e, tolerance=0.5):
#         print(f"正在飞往 N:{target_n}, E:{target_e} ...")
#         while True:
#             # 获取当前位置 (你需要订阅 LOCAL_POSITION_NED 消息)
#             msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
#             current_n = msg.x
#             current_e = msg.y
#
#             # 计算距离平方 (避免开根号运算，稍微快一点点，虽然Python并不在乎)
#             dist_sq = (target_n - current_n) ** 2 + (target_e - current_e) ** 2
#
#             # 实时打印距离，方便你看着爽
#             print(f"距离目标还有: {math.sqrt(dist_sq):.2f} m")
#
#             if dist_sq < (tolerance ** 2):
#                 print(">>> 已到达路点! <<<")
#                 break
#
#             time.sleep(0.2)
#
#     def check_and_sow(self):
#         print("\n🌱     播种中...      \n")
#
#     def return_to_launch(self):
#         self.master.mav.set_position_target_local_ned_send(
#             0,  # boot_time
#             self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
#             0b110111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
#             0, 0, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
#             0, 0, 0,  # 速度 (忽略)
#             0, 0, 0,  # 加速度 (忽略)
#             0, 0)  # 偏航角 (忽略)
#         self.wait_until_arrived(0, 0)
#         # --- 扩充动作 3：自动降落 (Land) ---
#         print("动作：开始自动降落")
#         self.master.mav.command_long_send(
#             self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_LAND,
#             0, 0, 0, 0, 0, 0, 0, 0)
#         # 监控高度直到着陆
#         while True:
#             msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#             alt_m = msg.relative_alt / 1000.0
#             print(f"降落中... 当前高度: {alt_m:.2f} 米")
#             if alt_m < 0.3:
#                 print("已着陆，电机锁定。")
#                 break
#
#         print("\n-播种任务结束-")


# ==========================================
# 主程序入口
# ==========================================

# ==========================================
# 模块3: 飞控通信 (MAVLink + 队列执行)
# ==========================================
class DroneController:
    def __init__(self, connection_str, gcs_ip='127.0.0.1', gcs_port=14550):
        print(f"🔗 [系统] 连接飞控接口: {connection_str}")

        # --- 1. 建立飞控连接 ---
        try:
            self.master = mavutil.mavlink_connection(connection_str, baud=57600, autoreconnect=True)
            print("⏳ [系统] 等待飞控心跳...")
            self.master.wait_heartbeat()
            print("💓 [系统] 收到飞控心跳!")

            # 请求数据流
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
            )
        except Exception as e:
            print(f"❌ [错误] 飞控连接失败: {e}")
            sys.exit(1)

        # --- 2. 建立地面站 UDP 连接 ---
        self.gcs_addr = (gcs_ip, gcs_port)
        self.gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gcs_socket.settimeout(0.5)  # 设置超时防阻塞
        print(f"📡 [系统] 地面站转发就绪 -> UDP {gcs_ip}:{gcs_port}")

        # --- 3. 初始化状态缓存 (解决线程竞争冲突) ---
        self.transformer = None
        self.cached_meridian = None

        # 这是一个线程安全的字典，用于存储最新的无人机状态
        # 这样转发线程在读取消息时，顺便更新这里，主线程直接读这里即可
        self.drone_state = {
            'lon': 0,
            'lat': 0,
            'alt': 0,  # 相对高度 (m)
            'last_update': 0  # 用于判断数据是否新鲜
        }

        # --- 4. 启动后台双线程 ---
        self.stop_event = threading.Event()

        # 线程A: 飞控 -> 地面站 (兼顾状态更新)
        self.t_fc = threading.Thread(target=self._fc_to_gcs_loop)
        self.t_fc.daemon = True
        self.t_fc.start()

        # 线程B: 地面站 -> 飞控
        self.t_gcs = threading.Thread(target=self._gcs_to_fc_loop)
        self.t_gcs.daemon = True
        self.t_gcs.start()

    # ==========================
    #      后台线程逻辑
    # ==========================
    def _fc_to_gcs_loop(self):
        """
        线程A: 读取串口 -> 转发UDP + 更新内部状态缓存
        """
        while not self.stop_event.is_set():
            try:
                # 1. 读取消息 (这是唯一的读取入口!)
                msg = self.master.recv_match(blocking=True, timeout=0.1)

                if not msg:
                    continue

                # 2. 【转发】发送二进制流给地面站
                try:
                    self.gcs_socket.sendto(msg.get_msgbuf(), self.gcs_addr)
                except:
                    pass  # UDP 发送失败不应影响主逻辑

                # 3. 【监听】如果是位置消息，更新缓存
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.drone_state['lon'] = msg.lon / 1e7
                    self.drone_state['lat'] = msg.lat / 1e7
                    self.drone_state['alt'] = msg.relative_alt / 1000.0
                    self.drone_state['last_update'] = time.time()

                # (可选) 也可以监听心跳来更新连接状态

            except Exception as e:
                # 防止单次读取错误搞崩线程
                pass

    def _gcs_to_fc_loop(self):
        """
        线程B: 监听UDP -> 写入串口
        """
        while not self.stop_event.is_set():
            try:
                data, addr = self.gcs_socket.recvfrom(4096)
                if data:
                    self.master.write(data)
            except socket.timeout:
                continue
            except Exception as e:
                time.sleep(0.1)

    # ==========================
    #      控制与计算逻辑
    # ==========================

    def _calculate_gauss_distance(self, lon1, lat1, lon2, lat2):
        # ... (保持之前的优化版代码不变) ...
        current_meridian = round(lon1 / 3) * 3
        if self.transformer is None or self.cached_meridian != current_meridian:
            proj_str = f"+proj=tmerc +lat_0=0 +lon_0={current_meridian} +k=1 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m"
            self.transformer = Transformer.from_crs("EPSG:4326", proj_str, always_xy=True)
            self.cached_meridian = current_meridian

        x1, y1 = self.transformer.transform(lon1, lat1)
        x2, y2 = self.transformer.transform(lon2, lat2)
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_current_position(self):
        """
        安全地获取当前位置 (无需再 recv_match)
        """
        # 检查数据是否太旧 (超过2秒没更新说明GPS可能丢了)
        # if time.time() - self.drone_state['last_update'] > 5.0:
        #     print("⚠️ [警告] GPS数据超时/未就绪!")
        #     return None
        return self.drone_state['lon'], self.drone_state['lat'], self.drone_state['alt']

    def takeoff(self, target_alt=10):
        print(f"🚀 [起飞] 目标高度: {target_alt}米")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_alt)

        # 阻塞监控 (改为读取缓存)
        while True:
            pos = self.get_current_position()
            if pos:
                _, _, curr_alt = pos
                sys.stdout.write(f"\r📈 当前高度: {curr_alt:.2f} m")
                sys.stdout.flush()

                if curr_alt >= target_alt * 0.95:
                    print("\n✅ [起飞] 到达目标高度，悬停中...")
                    break
            time.sleep(0.2)

    def fly_mission(self, waypoint_queue):
        print(f"📜 [任务] 开始执行，共有 {len(waypoint_queue)} 个航点")
        while len(waypoint_queue) > 0:
            target = waypoint_queue.popleft()
            # 这里的 target 可能是 (lon, lat) 或 (lon, lat, alt)
            # 兼容处理
            if len(target) == 3:
                t_lon, t_lat, t_alt = target
            else:
                t_lon, t_lat = target
                t_alt = 10  # 默认高度

            print(f"\n👉 [导航] 前往航点: {t_lon:.6f}, {t_lat:.6f}, H:{t_alt}m")
            self.send_reposition_command(t_lon, t_lat, t_alt)
            self.wait_until_arrived_gps(t_lon, t_lat, t_alt)
            self.check_and_sow()
        print("🏁 [任务] 所有航点执行完毕!")

    def send_reposition_command(self, lon, lat, alt, speed=50):
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        self.master.mav.command_int_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, speed, mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            0, 0, lat_int, lon_int, float(alt)
        )

    def wait_until_arrived_gps(self, target_lon, target_lat, target_alt, tolerance=1.5):
        """
        基于缓存状态的到达判断
        """
        while True:
            # 1. 读缓存
            pos = self.get_current_position()
            if not pos:
                time.sleep(0.5)
                continue

            curr_lon, curr_lat, curr_alt = pos

            # 2. 算距离 (使用优化后的投影)
            h_dist = self._calculate_gauss_distance(curr_lon, curr_lat, target_lon, target_lat)

            sys.stdout.write(f"\r📍 距目标: {h_dist:.2f}m | 高度: {curr_alt:.1f}m")
            sys.stdout.flush()

            if h_dist < tolerance:
                print(f"\n✅ [导航] 到达航点! (误差: {h_dist:.2f}m)")
                break

            time.sleep(0.2)  # 不用太快，给转发线程留CPU

    def check_and_sow(self):
        print("🌱 [作业] 正在播种...")
        time.sleep(1)

    def return_to_launch(self):
        print("\n🏠 [返航] 触发 RTL...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0)

        time.sleep(2)
        while True:
            pos = self.get_current_position()
            if pos:
                _, _, alt_m = pos
                sys.stdout.write(f"\r🛬 降落中... 高度: {alt_m:.2f} m")
                sys.stdout.flush()
                if alt_m < 0.3:
                    print("\n✅ [结束] 已着陆.")
                    break
            time.sleep(0.5)

    def close(self):
        # 程序退出时调用，停止线程
        self.stop_event.set()
        self.gcs_socket.close()
        self.master.close()


# ==========================================
# 模块4: 辅助方法:生成虚拟作业区域
# ==========================================
def generate_fake_polygons(home_lon, home_lat, min_dist=30, max_dist=500, count_range=(3, 5)):
    """
    在起飞点附近随机生成若干个测试用的 polygon 区域 (经纬度格式).

    :param home_lon: 起飞点经度
    :param home_lat: 起飞点纬度
    :param min_dist: 区域中心距离起飞点的最小距离 (米)
    :param max_dist: 区域中心距离起飞点的最大距离 (米)
    :param count_range: 生成数量范围 (min, max)
    :return: 一个包含 shapely.geometry.box 的列表
    """
    polygons = []
    num_polys = random.randint(*count_range)

    print(f"🎲 [仿真] 正在生成 {num_polys} 个随机任务区域 (距离 {min_dist}-{max_dist}m)...")

    # --- 核心换算逻辑 ---
    # 地球半径约为 6378137 米
    # 纬度 1度 ≈ 111132 米
    # 经度 1度 ≈ 111132 * cos(纬度) 米
    m_per_deg_lat = 111132
    m_per_deg_lon = 111132 * math.cos(math.radians(home_lat))

    for i in range(num_polys):
        # 1. 随机生成相对起飞点的距离 (米) 和方位角
        distance = random.uniform(min_dist, max_dist)
        angle_deg = random.uniform(0, 360)
        angle_rad = math.radians(angle_deg)

        # 2. 计算中心点的偏移量 (米 -> 经纬度差)
        delta_x_meters = distance * math.cos(angle_rad)  # 东向偏移
        delta_y_meters = distance * math.sin(angle_rad)  # 北向偏移

        center_lon = home_lon + (delta_x_meters / m_per_deg_lon)
        center_lat = home_lat + (delta_y_meters / m_per_deg_lat)

        # 3. 随机生成矩形的大小 (例如边长 20m 到 60m 的区域)
        width_m = random.uniform(20, 60)
        height_m = random.uniform(20, 60)

        # 4. 将矩形宽高也转换为经纬度差
        delta_w = (width_m / 2) / m_per_deg_lon
        delta_h = (height_m / 2) / m_per_deg_lat

        # 5. 生成 shapely box 对象
        # box(minx, miny, maxx, maxy) -> (min_lon, min_lat, max_lon, max_lat)
        poly = box(
            center_lon - delta_w,
            center_lat - delta_h,
            center_lon + delta_w,
            center_lat + delta_h
        )
        polygons.append(poly)

        # (可选) 打印一下生成结果方便调试
        # print(f"  👉 区域{i+1}: 距家 {distance:.1f}m, 中心 ({center_lon:.6f}, {center_lat:.6f})")

    return polygons


if __name__ == "__main__":
    # DEM 文件路径
    # dem_file = r"C:\Users\Y9000\Desktop\工作文档\无人机\测试数据-毛\测试dem.tif"
    # demAnalyzer = DemAnalyzer(dem_file)
    # true_polygon = demAnalyzer.detect_suitable_seeding_zones()

    # # 模拟几个多边形(Valid Zones)代替 DEM 解析结果，方便你直接测试逻辑
    # fake_polygons = [
    #     box(10, 10, 30, 30),  # 20x20的区域
    #     box(50, 50, 70, 60),  # 20x10的区域
    #     box(-20, 10, -10, 40),  # 长条区域
    # ]

    # 2. 路径规划
    my_home_lon = -122.3895140
    my_home_lat = 37.62785727

    # 替换之前的 fake_polygons = [box(10, 10, 30, 30)]
    fake_polygons = generate_fake_polygons(my_home_lon, my_home_lat)
    print(f"\n✅ 生成完毕! 共 {len(fake_polygons)} 个区域")
    for idx, poly in enumerate(fake_polygons):
        # 打印一下它的 WKT 字符串看看样子
        print(f"🔸 区域 {idx + 1} Bounds: {poly.bounds}")

    planner = SmartPathPlanner(swath_width=5.0)
    current_drone_pos = (my_home_lon, my_home_lat)  # 本地坐标原点

    # 将多个区域的"蛇形路径"通过贪心算法串联成一个大队列
    mission_queue = planner.sort_regions_greedy(current_drone_pos, fake_polygons)

    print(f"路径规划完成! 总航点数: {len(mission_queue)}")

    # 3. 飞控执行
    # Windows仿真连接串: 'tcp:127.0.0.1:5762'
    drone = DroneController('COM7')

    # drone.fly_mission(mission_queue)
    # drone.return_to_launch()
    try:
        input("按回车起飞...")
        drone.takeoff(10)
        # 这里你可以接入你的 generate_fake_polygons 和 SmartPathPlanner 生成的航点
        drone.fly_mission(mission_queue)
        drone.return_to_launch()
    except KeyboardInterrupt:
        print("手动中断")
    finally:
        drone.close()
