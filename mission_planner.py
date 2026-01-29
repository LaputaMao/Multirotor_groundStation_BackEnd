import os
import sys

import numpy as np
import rasterio
from rasterio.features import shapes
from shapely.geometry import shape, LineString, Polygon, box
from shapely.ops import unary_union
from pymavlink import mavutil
import math
import time
from collections import deque
from scipy.ndimage import uniform_filter


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


# ==========================================
# 模块2: 路径规划 (切条 + 贪心算法)
# ==========================================
class SmartPathPlanner:
    def __init__(self, swath_width=5.0):
        self.swath_width = swath_width  # 航道宽度 5米

    # def generate_snake_path(self, polygon):
    #     """
    #     全覆盖路径算法：将多边形切成蛇形航点
    #     """
    #     minx, miny, maxx, maxy = polygon.bounds
    #     path_points = []
    #
    #     # 从下往上，每隔5米切一刀
    #     scan_line_y = miny + (self.swath_width / 2)
    #     direction = 1  # 1: 向右飞, -1: 向左飞 (实现蛇形)
    #
    #     while scan_line_y < maxy:
    #         # 创建一条水平线
    #         line = LineString([(minx, scan_line_y), (maxx, scan_line_y)])
    #         # 求交集（航道在多边形内的部分）
    #         intersection = line.intersection(polygon)
    #
    #         if not intersection.is_empty:
    #             # 处理可能产生的多段线（如果地形是U型）
    #             if intersection.geom_type == 'MultiLineString':
    #                 segs = list(intersection.geoms)
    #             else:
    #                 segs = [intersection]
    #
    #             # 对线段进行排序（根据当前飞行方向）
    #             coords = []
    #             for seg in segs:
    #                 c = list(seg.coords)
    #                 if direction == -1:
    #                     c.reverse()
    #                 coords.extend(c)
    #
    #             # 如果是整条线反向（蛇形逻辑）
    #             if direction == -1:
    #                 coords.reverse()
    #
    #             path_points.extend(coords)
    #
    #         scan_line_y += self.swath_width
    #         direction *= -1  # 下一行反向
    #
    #     return path_points  # 返回 [(x,y), (x,y)...]
    def generate_snake_path(self, polygon):
        """
        全覆盖路径算法 (修正版)：严格的蛇形 (Boustrophedon)
        偶数行：左 -> 右
        奇数行：右 -> 左
        """
        minx, miny, maxx, maxy = polygon.bounds
        path_points = []

        # 1. 确定所有的扫描线 Y 坐标
        scan_ys = []
        y = miny + (self.swath_width / 2)
        while y < maxy:
            scan_ys.append(y)
            y += self.swath_width

        # 2. 遍历每一行
        for i, current_y in enumerate(scan_ys):
            # 构造水平扫描线
            line = LineString([(minx, current_y), (maxx, current_y)])
            intersection = line.intersection(polygon)

            if intersection.is_empty:
                continue

            # 3. 标准化线段 (处理可能出现的多段线，比如 U 型地形)
            if intersection.geom_type == 'MultiLineString':
                segs = list(intersection.geoms)
            else:
                segs = [intersection]

            # --- 关键步骤 A: 强制按 X 轴从小到大排序 ---
            # 无论 shapely 怎么返回，我们先把线段按“从西向东”排好序
            # 这样 base_coords 永远是：[(x_min, y), (x_mid, y)... (x_max, y)]
            segs.sort(key=lambda s: s.bounds[0])

            base_coords = []
            for seg in segs:
                # 确保每一小段也是无脑从左到右
                # (shapely 的 coords 只有两个点，但也得防一手)
                seg_coords = list(seg.coords)
                if seg_coords[0][0] > seg_coords[-1][0]:
                    seg_coords.reverse()
                base_coords.extend(seg_coords)

            # --- 关键步骤 B: 根据行号(i) 决定是否翻转整个列表 ---
            # 偶数行 (0, 2, 4...) : 保持 (左 -> 右)
            # 奇数行 (1, 3, 5...) : 翻转 (右 -> 左)
            if i % 2 == 1:
                base_coords.reverse()

            path_points.extend(base_coords)

        return path_points

    def sort_regions_greedy(self, start_pos, polygons):
        """
        贪心算法排序：决定先飞哪个区域
        """
        current_pos = start_pos
        remaining_polys = polygons.copy()
        final_queue = deque()  # 最终的坐标队列

        print("正在规划全局飞行顺序...")

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
                dist = math.hypot(start_node[0] - current_pos[0], start_node[1] - current_pos[1])

                if dist < best_dist:
                    best_dist = dist
                    best_poly_idx = i
                    best_path = path

            # 选中了最近的区域
            if best_poly_idx != -1:
                # 将该区域的所有点加入总队列
                for p in best_path:
                    final_queue.append(p)

                # 更新当前位置为该区域的最后一个点
                current_pos = best_path[-1]
                # 移除该区域
                remaining_polys.pop(best_poly_idx)
            else:
                break  # 应该不会发生，除非所有区域都生成路径失败

        return final_queue


# ==========================================
# 模块3: 飞控通信 (MAVLink + 队列执行)
# ==========================================
class DroneController:
    def __init__(self, connection_str):
        print(f"连接飞控: {connection_str}")
        # self.master = mavutil.mavlink_connection('COM7', 57600, autoreconnect=True)
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        print("正在请求数据流...")
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # 请求所有类型的数据(或者用 MAV_DATA_STREAM_POSITION)
            1,  # 请求频率 (Hz)，这里设为 4Hz (每秒4次) 足够了
            1  # 开启 (start)
        )

        print("飞控已连接!")
        self.current_pos = (0, 0, 0)  # N, E, D

    def change_mode(self, mode_name):
        # 获取飞控目前支持的模式ID
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"不支持的模式: {mode_name}")
            sys.exit(1)

        print(f"正在切换到 {mode_name} 模式...")
        # 发送设置模式指令
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        # self.master.mav.command_long_send(
        #     self.master.target_system,
        #     self.master.target_component,
        #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command 176
        #     0,  # confirmation
        #     1,  # param1: 开启 Custom Mode (必填1)
        #     18,  # param2: 目标模式 ID
        #     0, 0, 0, 0, 0  # param3-7: 未使用
        # )

        # 循环等待，直到模式真的变过来
        while True:
            # 监听心跳包来确认模式
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
            # 检查当前模式是否已变更
            if msg.custom_mode == mode_id:
                print(f"模式已切换为: {mode_name}")
                break
            time.sleep(0.1)

    def takeoff(self, alt=10):
        # 起飞代码
        print("正在解锁...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("已解锁!")
        print(f"起飞至 {alt}米...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
        # time.sleep(5)  # 简单等待起飞
        # 监控起飞状态
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                # 这里的 relative_alt 单位是毫米，所以要除以1000
                current_alt = msg.relative_alt / 1000.0
                print(f"当前高度: {current_alt:.2f} m")

                if current_alt >= alt * 0.95:
                    print("到达目标高度! 悬停中...")
                    break

            time.sleep(0.5)

    def fly_mission(self, waypoint_queue):
        """
        核心执行逻辑：从队列取点 -> 飞 -> 等 -> 下一个
        """
        print(f"开始执行任务，共有 {len(waypoint_queue)} 个航点")

        while len(waypoint_queue) > 0:
            # 1. 从队列取出下一个点 (FIFO)
            target = waypoint_queue.popleft()  # (x, y) 假设是投影坐标或NED

            # 注意：如果DEM坐标是投影坐标(米)，你需要根据起飞点转换成相对坐标(NED)
            # 这里假设 target 已经是相对于起飞点的 NED 坐标 (North, East)
            target_n, target_e = target

            print(f">>> 前往航点: N {target_n:.1f}, E {target_e:.1f}")

            # 2. 发送指令
            self.send_position_target(target_n, target_e, -10)  # 保持高度10米

            # 3. 阻塞等待到达
            self.wait_until_arrived(target_n, target_e)

            # 4. 到达后执行动作，比如播种机开
            self.check_and_sow()

        print("所有航点执行完毕!")

    def send_position_target(self, n, e, d):
        # ... 你之前的 set_position_target_local_ned_encode 代码 ...
        # 注意 type_mask 要忽略速度和加速度，只控制位置
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,  # 使用位置控制
            n, e, d,
            0, 0, 0, 0, 0, 0, 0, 0)

    # NED 坐标系距离检查函数
    def wait_until_arrived(self, target_n, target_e, tolerance=0.5):
        print(f"正在飞往 N:{target_n}, E:{target_e} ...")
        while True:
            # 获取当前位置 (你需要订阅 LOCAL_POSITION_NED 消息)
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            current_n = msg.x
            current_e = msg.y

            # 计算距离平方 (避免开根号运算，稍微快一点点，虽然Python并不在乎)
            dist_sq = (target_n - current_n) ** 2 + (target_e - current_e) ** 2

            # 实时打印距离，方便你看着爽
            print(f"距离目标还有: {math.sqrt(dist_sq):.2f} m")

            if dist_sq < (tolerance ** 2):
                print(">>> 已到达路点! <<<")
                break

            time.sleep(0.2)

    def check_and_sow(self):
        print("\n🌱     播种中...      \n")

    def return_to_launch(self):
        self.master.mav.set_position_target_local_ned_send(
            0,  # boot_time
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用本地坐标系
            0b110111111000,  # 掩码：只保留位置信息 (忽略速度和加速度)
            0, 0, -10,  # X(北), Y(东), Z(下，高度10米所以是-10)
            0, 0, 0,  # 速度 (忽略)
            0, 0, 0,  # 加速度 (忽略)
            0, 0)  # 偏航角 (忽略)
        self.wait_until_arrived(0, 0)
        # --- 扩充动作 3：自动降落 (Land) ---
        print("动作：开始自动降落")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0)
        # 监控高度直到着陆
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt_m = msg.relative_alt / 1000.0
            print(f"降落中... 当前高度: {alt_m:.2f} 米")
            if alt_m < 0.3:
                print("已着陆，电机锁定。")
                break

        print("\n-播种任务结束-")


# ==========================================
# 主程序入口
# ==========================================
if __name__ == "__main__":
    # DEM 文件路径
    # dem_file = r"C:\Users\Y9000\Desktop\工作文档\无人机\测试数据-毛\测试dem.tif"
    # demAnalyzer = DemAnalyzer(dem_file)
    # true_polygon = demAnalyzer.detect_suitable_seeding_zones()

    # 模拟几个多边形(Valid Zones)代替 DEM 解析结果，方便你直接测试逻辑
    fake_polygons = [
        box(10, 10, 30, 30),  # 20x20的区域
        box(50, 50, 70, 60),  # 20x10的区域
        box(-20, 10, -10, 40),  # 长条区域
    ]

    # 2. 路径规划
    planner = SmartPathPlanner(swath_width=5.0)
    current_drone_pos = (0, 0)  # 本地坐标原点

    # 将多个区域的"蛇形路径"通过贪心算法串联成一个大队列
    mission_queue = planner.sort_regions_greedy(current_drone_pos, fake_polygons)

    print(f"路径规划完成! 总航点数: {len(mission_queue)}")

    # 3. 飞控执行
    # Windows仿真连接串: 'tcp:127.0.0.1:5762'
    drone = DroneController('tcp:127.0.0.1:5762')

    # 这里加个输入，防止一连上就飞
    input("按回车键开始起飞执行任务...")

    drone.change_mode('GUIDED')
    drone.takeoff(10)
    drone.fly_mission(mission_queue)
    drone.return_to_launch()
