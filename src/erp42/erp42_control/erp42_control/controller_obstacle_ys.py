import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from scipy.interpolate import CubicSpline
from scipy.cluster.hierarchy import linkage, fcluster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from shapely.geometry import Point, Polygon, LineString
from geometry_msgs.msg import Point as ROSPoint
from stanley import Stanley
from rclpy.qos import qos_profile_system_default, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from erp42_msgs.msg import ControlMessage
import time as t

# ★ ref를 DB에서 읽기 위해 추가
from DB import DB


class SpeedSupporter:
    def __init__(self, node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_obstacle", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_obstacle", 30.0).value
        self.he_thr  = node.declare_parameter("/speed_supporter/he_thr_obstacle", 0.001).value
        self.ce_thr  = node.declare_parameter("/speed_supporter/ce_thr_obstacle", 0.002).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res


class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_obstacle", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_obstacle", 0.85).value
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        now = node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        self.last    = self.current

    def PIDControl(self, speed, desired_value):
        now = self.node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        dt = max(self.current - self.last, 1e-3)   # dt 보정
        self.last = self.current

        err = desired_value - speed
        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, 4, 6))


class Obstacle:
    def __init__(self, node):
        self.node = node
        self.sub_marker = self.node.create_subscription(MarkerArray, "/markers", self.call_marker, 10)

        # ----- 기존 Path 퍼블리셔(다른 노드가 쓸 수 있으므로 유지) -----
        self.ref_path1 = self.node.create_publisher(Path, "/ref/path1", 10)
        self.ref_path2 = self.node.create_publisher(Path, "/ref/path2", 10)
        self.LocalPath_pub = self.node.create_publisher(Path, "/path/avoid_path", 10)

        # ----- RViz용 통합 MarkerArray 퍼블리셔 (/obstacle_viz) -----
        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,   # RViz 나중에 켜도 마지막 메시지 유지
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.viz_pub = self.node.create_publisher(MarkerArray, "/obstacle_viz", viz_qos)

        # 기존 개별 시각화 퍼블리셔는 더 이상 사용하지 않음
        # self.marker_pub = self.node.create_publisher(MarkerArray, "transformed_markers", 10)
        # self.road_poly  = self.node.create_publisher(Marker, "visualization_marker", 10)

        self.odometry = None
        self.odom_pose = np.array([0.0, 0.0])
        self.odom_orientation = [0.0, 0.0, 0.0, 1.0]

        # 장애물
        self.obs = np.array([]).reshape(0, 2)
        self.num1_obs = []
        self.num2_obs = []

        # 경로/상태
        self.ref_path_points1 = None
        self.ref_path_points2 = None
        self.local_points = None
        self.local_x, self.local_y, self.local_yaw = [], [], []
        self.st = Stanley()
        self.pid = PID(node)
        self.ss  = SpeedSupporter(node)

        self.code_start = True
        self.state = "dynamic"  # dynamic, static
        self.change_time = None
        self.estop = 0
        self.observed_points = []
        self.o_list = [0] * 30 # 장애물 사라지고 난후 estop off 하는 시간
        self.once = 0
        self.to_num = None

        # DB 로딩
        self.path1_db = DB("tunnel_path/obs_path_1.db")
        self.path2_db = DB("tunnel_path/obs_path_2.db")
        self.path1_data = np.array(self.path1_db.read_db_n("Path", "x", "y", "yaw"))
        self.path2_data = np.array(self.path2_db.read_db_n("Path", "x", "y", "yaw"))

        if self.path1_data.size == 0:
            self.node.get_logger().warn("[DB] lane1(Path1) 데이터가 비어있습니다: obs_path_1.db")
        if self.path2_data.size == 0:
            self.node.get_logger().warn("[DB] lane2(Path2) 데이터가 비어있습니다: obs_path_2.db")

        # 탐지 파라미터
        self.win_len = node.declare_parameter("/detection/window_length_m", 30.0).value # ROI 넓이 조정 parameter
        self.pad1    = node.declare_parameter("/detection/padding_lane1_m", 1.2).value
        self.pad2    = node.declare_parameter("/detection/padding_lane2_m", 1.2).value

        # 내부 보관(폴리곤 외곽 좌표 → RViz 마커에서 사용)
        self._lane1_poly_coords = None
        self._lane2_poly_coords = None

    # ------------------- RViz 통합 마커 빌드 -------------------
    def _mk_line_strip(self, pts_xy, ns, mid, rgba, width=0.08):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = float(width)
        m.color.r, m.color.g, m.color.b, m.color.a = [float(c) for c in rgba]
        m.pose.orientation.w = 1.0
        if pts_xy is not None:
            for x, y in pts_xy:
                p = ROSPoint()
                p.x, p.y, p.z = float(x), float(y), 0.0
                m.points.append(p)
            if len(pts_xy) > 0:
                # 닫히는 폴리곤(탐지 띠)일 경우, 첫 점을 다시 붙여 닫기
                if np.allclose(pts_xy[0], pts_xy[-1]) is False and ns.endswith("_DA"):
                    p0 = ROSPoint()
                    p0.x, p0.y, p0.z = float(pts_xy[0][0]), float(pts_xy[0][1]), 0.0
                    m.points.append(p0)
        return m

    def _mk_path_strip(self, pts_xy, ns, mid, rgba, width=0.05):
        # 경로는 닫지 않음
        return self._mk_line_strip(pts_xy, ns, mid, rgba, width)

    def _mk_sphere_list(self, pts_xy, ns, mid, rgba, scale=0.5):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = [float(c) for c in rgba]
        m.pose.orientation.w = 1.0
        if pts_xy:
            for x, y in pts_xy:
                p = ROSPoint()
                p.x, p.y, p.z = float(x), float(y), 0.0
                m.points.append(p)
        return m

    def _publish_viz(self):
        arr = MarkerArray()
        mid = 0

        # 1) DB 경로 1/2 (라인 스트립)
        if self.ref_path_points1 is not None:
            arr.markers.append(self._mk_path_strip(self.ref_path_points1, "lane1_path", mid, (0.0, 0.6, 0.0, 0.9), 0.06)); mid += 1
        if self.ref_path_points2 is not None:
            arr.markers.append(self._mk_path_strip(self.ref_path_points2, "lane2_path", mid, (0.6, 0.0, 0.0, 0.9), 0.06)); mid += 1

        # 2) 로컬 경로 (중간선, 파란색)
        if self.local_x and self.local_y:
            arr.markers.append(
                self._mk_path_strip(list(zip(self.local_x, self.local_y)), "local_path", mid, (0.0, 0.4, 1.0, 1.0), 0.08)
            ); mid += 1

        # 3) 탐지 폴리곤(라인 스트립)
        if self._lane1_poly_coords:
            arr.markers.append(self._mk_line_strip(self._lane1_poly_coords, "lane1_DA", mid, (0.0, 1.0, 0.0, 0.8), 0.10)); mid += 1
        if self._lane2_poly_coords:
            arr.markers.append(self._mk_line_strip(self._lane2_poly_coords, "lane2_DA", mid, (1.0, 0.0, 0.0, 0.8), 0.10)); mid += 1

        # 4) 장애물 점 (lane1: 초록, lane2: 빨강)
        if self.num1_obs:
            arr.markers.append(self._mk_sphere_list(self.num1_obs, "obs_lane1", mid, (0.0, 1.0, 0.0, 1.0), 0.5)); mid += 1
        if self.num2_obs:
            arr.markers.append(self._mk_sphere_list(self.num2_obs, "obs_lane2", mid, (1.0, 0.0, 0.0, 1.0), 0.5)); mid += 1

        # 퍼블리시 (TRANSIENT_LOCAL)
        self.viz_pub.publish(arr)

    # ------------------- 기존 로직 -------------------
    def call_marker(self, msg):
        if self.code_start:
            if self.odometry is not None and self.odometry.x is not None and self.odometry.y is not None:
                if msg is not None and hasattr(msg, "markers") and msg.markers:
                    markers = msg.markers
                    self.observed_points = []
                    for p in markers:
                        points = np.array([[pt.x, pt.y] for pt in p.points])
                        if len(points) > 0:
                            center = np.min(points, axis=0)  # 필요 시 np.mean으로 변경
                            transformed_center = self.transform_cluster_centers(np.array([center]))
                            self.observed_points.append(transformed_center[0])
                    if self.observed_points:
                        self.obs = np.array(self.observed_points)

    def rotate_points(self, points, angle):
        a = np.deg2rad(angle)
        R = np.array([[np.cos(a), -np.sin(a)],[np.sin(a), np.cos(a)]])
        return np.dot(points, R)

    def transform_cluster_centers(self, cluster_centers):
        if len(cluster_centers) == 0:
            return np.array([])
        _, _, yaw = euler_from_quaternion(self.odom_orientation)
        rotated_centers = self.rotate_points(cluster_centers, np.rad2deg(-yaw))
        transformed_centers = rotated_centers + np.array((self.odometry.x, self.odometry.y))
        return transformed_centers

    def organize_obstacle_lists(self):
        current_position = np.array([self.odometry.x, self.odometry.y])
        distances_num1 = [np.linalg.norm(np.array(p) - current_position) for p in self.num1_obs]
        best_point, best_distance, best_closest_distance = None, -1, float("inf")

        for i, p1 in enumerate(self.num1_obs):
            d_cur = distances_num1[i]
            closest = min([np.linalg.norm(np.array(p1) - np.array(p2)) for p2 in self.num2_obs], default=float("inf"))
            if closest < best_closest_distance or (closest == best_closest_distance and d_cur > best_distance):
                best_point, best_distance, best_closest_distance = p1, d_cur, closest

        if best_point is not None:
            self.num1_obs = [best_point]
        if self.num2_obs:
            d2 = [np.linalg.norm(np.array(p) - current_position) for p in self.num2_obs]
            idx = np.argsort(d2)[:3]
            self.num2_obs = [self.num2_obs[i] for i in idx]

    def publish_ref_path(self, wx, wy, num=None):
        cs_x = CubicSpline(range(len(wx)), wx); cs_y = CubicSpline(range(len(wy)), wy)
        distances = np.sqrt(np.diff(wx) ** 2 + np.diff(wy) ** 2); total_length = np.sum(distances)
        sampling = 0.1
        s = np.arange(0, len(wx) - 1, sampling / max(total_length, 1e-6) * (len(wx) - 1))
        rx = cs_x(s); ry = cs_y(s)
        path_points = np.vstack((rx, ry)).T

        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.position.z = 0.0
            yaw = np.arctan2(y - self.odometry.y, x - self.odometry.x)
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            path.poses.append(pose)

        if num == 1:
            self.ref_path1.publish(path); self.ref_path_points1 = path_points
        if num == 2:
            self.ref_path2.publish(path); self.ref_path_points2 = path_points

    def publish_ref_path_from_db(self, path_data: np.ndarray, num: int):
        if path_data is None or path_data.size == 0:
            self.node.get_logger().warn(f"[DB] lane{num} 경로 데이터 없음"); return
        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"
        pts = []
        for x, y, yaw in path_data:
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x); pose.pose.position.y = float(y); pose.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, float(yaw))
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            path.poses.append(pose); pts.append((float(x), float(y)))
        if num == 1:
            self.ref_path1.publish(path); self.ref_path_points1 = np.array(pts)
        elif num == 2:
            self.ref_path2.publish(path); self.ref_path_points2 = np.array(pts)

    def publish_local_path(self, points):
        local_x, local_y, local_yaw = [], [], []
        for x, y in points:
            local_x.append(x); local_y.append(y)
        dx = np.diff(local_x); dy = np.diff(local_y)
        distances = np.sqrt(dx**2 + dy**2)
        if len(distances) == 0:
            return
        total_distance = np.cumsum(distances)
        total_distance = np.insert(total_distance, 0, 0)
        interp_distances = np.arange(0, total_distance[-1], 0.1)
        interp_x = np.interp(interp_distances, total_distance, local_x)
        interp_y = np.interp(interp_distances, total_distance, local_y)

        path = Path(); path.header.frame_id = "map"
        for i in range(len(interp_x) - 1):
            x, y = interp_x[i], interp_y[i]
            nx, ny = interp_x[i+1], interp_y[i+1]
            yaw = np.arctan2(ny - y, nx - x)
            local_yaw.append(yaw)
            pose = PoseStamped()
            pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.position.z = -1.0
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            path.poses.append(pose)

        self.LocalPath_pub.publish(path)
        self.local_x = interp_x.tolist(); self.local_y = interp_y.tolist(); self.local_yaw = local_yaw

    def line_change(self):
        if len(self.num2_obs) > 1:
            for obs_x, obs_y in self.num2_obs:
                _, _, yaw = euler_from_quaternion(self.odom_orientation)
                obs_length = sqrt((obs_x - self.odometry.x) ** 2 + (obs_y - self.odometry.y) ** 2)
                if len(self.num1_obs) < 1:
                    if obs_length <= 2.5:
                        print("긴급회피"); self.to_num = 1
                        self.local_points = self.ref_path_points1.tolist()
                        self.publish_local_path(self.local_points); self.change_time = t.time(); return
                else:
                    if obs_length <= 3:
                        print("회피"); self.to_num = 1
                        self.local_points = self.ref_path_points1.tolist()
                        self.publish_local_path(self.local_points); self.change_time = t.time(); return

        if self.to_num is None:
            self.local_points = self.ref_path_points2.tolist()
        self.to_num = 2
        self.publish_local_path(self.local_points)

    # ---- DB 경로 기반 DA 생성 ----
    def _closest_index(self, path_xy: np.ndarray, pos_xy: np.ndarray) -> int:
        if path_xy is None or len(path_xy) == 0:
            return 0
        d = np.linalg.norm(path_xy - pos_xy[None, :], axis=1)
        return int(np.argmin(d))

    def _forward_slice(self, path_xy: np.ndarray, start_idx: int, length_m: float) -> np.ndarray:
        if path_xy is None or len(path_xy) < 2:
            return path_xy
        pts = [path_xy[start_idx]]
        acc = 0.0
        for i in range(start_idx, len(path_xy) - 1):
            seg = np.linalg.norm(path_xy[i + 1] - path_xy[i])
            pts.append(path_xy[i + 1]); acc += seg
            if acc >= length_m: break
        return np.vstack(pts)

    def _build_da_polygon(self, path_data_xy: np.ndarray, padding_m: float, window_len_m: float):
        if path_data_xy is None or path_data_xy.size < 4:
            return None, None
        start_idx = self._closest_index(path_data_xy, np.array([self.odometry.x, self.odometry.y]))
        seg_xy = self._forward_slice(path_data_xy, start_idx, window_len_m)
        if seg_xy is None or len(seg_xy) < 2:
            return None, None
        line = LineString(seg_xy.tolist())
        poly = line.buffer(padding_m, cap_style=2, join_style=2)
        coords = list(poly.exterior.coords)
        return poly, coords

    def check_obstacle_db(self):
        lane1_xy = self.path1_data[:, :2] if (self.path1_data is not None and self.path1_data.size >= 2) else None
        lane2_xy = self.path2_data[:, :2] if (self.path2_data is not None and self.path2_data.size >= 2) else None
        poly1, coords1 = self._build_da_polygon(lane1_xy, self.pad1, self.win_len) if lane1_xy is not None else (None, None)
        poly2, coords2 = self._build_da_polygon(lane2_xy, self.pad2, self.win_len) if lane2_xy is not None else (None, None)

        # 외곽 좌표를 저장해 두고 RViz에서 한 번에 그림
        self._lane1_poly_coords = coords1
        self._lane2_poly_coords = coords2

        self.num1_obs, self.num2_obs = [], []
        if self.obs is not None and len(self.obs) > 0:
            for ox, oy in self.obs:
                p = Point(ox, oy)
                if poly1 is not None and poly1.contains(p):
                    self.num1_obs.append((ox, oy))
                if poly2 is not None and poly2.contains(p):
                    self.num2_obs.append((ox, oy))

    def stop_to(self):
        # (필요 시) 동적 estop 폴리곤 마커도 통합 배열에 얹고 싶다면 이 함수에서 좌표를 저장해
        polygon_points = [
            (73.94788360595703, -91.37284851074219),
            (71.81258392333984, -86.93596649169922),
            (175.8633270263672, -26.53049087524414),
            (177.93630981445312, -30.269697189331055),
        ]
        polygon = Polygon(polygon_points)
        # RViz 표시를 원하면 다음과 같이 coords를 저장하고 _publish_viz에서 그리면 됨
        # self._stop_poly_coords = list(polygon.exterior.coords)

        if self.observed_points:
            for obs_point in self.observed_points:
                point = Point(obs_point)
                self.o_list.append(1 if polygon.contains(point) else 0); del self.o_list[0]
            self.observed_points = []
        else:
            self.o_list.append(0); del self.o_list[0]

        if 1 in self.o_list:
            self.estop = 1; self.once = 1
        else:
            self.estop = 0
            if self.once == 1:
                self.once = 0; self.change_time = t.time(); self.obs = []

    def control_obstacle(self, odometry, path):
        self.odometry = odometry
        self.timer_callback()
        msg = ControlMessage()

        if len(self.local_x) != 0:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry, self.local_x, self.local_y, self.local_yaw, h_gain=1.0, c_gain=0.8
            )
            target_speed = 15.0
            adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=10, max_value=15)
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
        else:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry, path.cx, path.cy, path.cyaw, h_gain=1.0, c_gain=0.8
            )
            target_speed = 15.0
            adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=10, max_value=15)
            speed = self.pid.PIDControl(self.odometry.v * 3.6, adapted_speed)
            

        msg.speed = int(speed) * 10
        msg.steer = int(degrees((-1) * steer))
        msg.gear = 2
        msg.estop = self.estop

        if self.target_idx >= (len(self.local_points) - 10 if self.local_points else 0):
            self.to_num = None; self.code_start = False
            self.num1_obs = []; self.num2_obs = []
            return msg, True
        else:
            return msg, False

    def timer_callback(self):
        if not self.code_start:
            self.code_start = True

        self.odom_pose = np.array([self.odometry.x, self.odometry.y])
        q = quaternion_from_euler(0, 0, self.odometry.yaw)
        self.odom_orientation = [q[0], q[1], q[2], q[3]]

        if self.odometry is not None and self.odometry.x is not None and self.odometry.y is not None:
            if self.state == "static":
                if self.path1_data.size >= 3: self.publish_ref_path_from_db(self.path1_data, num=1)
                else: self.node.get_logger().warn("[DB] lane1 데이터 부족")
                if self.path2_data.size >= 3: self.publish_ref_path_from_db(self.path2_data, num=2)
                else: self.node.get_logger().warn("[DB] lane2 데이터 부족")

                self.check_obstacle_db()
                self.organize_obstacle_lists()
                self.line_change()

            elif self.state == "dynamic":
                if self.path1_data.size >= 3: self.publish_ref_path_from_db(self.path1_data, num=1)
                if self.path2_data.size >= 3: self.publish_ref_path_from_db(self.path2_data, num=2)

                self.local_points = []
                for p1, p2 in zip(self.ref_path_points2, self.ref_path_points1):
                    self.local_points.append([(p1[0]+p2[0])/2.0, (p1[1]+p2[1])/2.0])

                self.publish_local_path(self.local_points)
                self.stop_to()
                if self.change_time is not None and (t.time() - self.change_time > 2):
                    self.state = "static"

        # === 마지막에 RViz 통합 마커 퍼블리시 ===
        self._publish_viz()
