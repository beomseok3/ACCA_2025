# controller_delivery.py

import math as m
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler

from DB import DB
from erp42_msgs.msg import ControlMessage
from stanley import Stanley
# YOLO 메시지 타입 (실제 타입으로 변경)
# from custom_msgs.msg import BoundingBoxes, BoundingBox


class SpeedSupporter:
    def __init__(self, node):
        self.he_gain = 50.0; self.ce_gain = 30.0
        self.he_thr  = 0.001; self.ce_thr  = 0.002

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        return np.clip(value + err, min_value, max_value)


class PID:
    def __init__(self, node):
        self.node   = node
        self.p_gain = 2.07; self.i_gain = 0.85
        self.p_err = 0.0; self.i_err = 0.0
        now = node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        self.last    = self.current

    def PIDControl(self, speed, desired_value):
        now = self.node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        dt = self.current - self.last
        self.last = self.current
        err = desired_value - speed
        self.p_err = err
        self.i_err += err * dt * (0.0 if speed == 0 else 1.0)
        out = speed + self.p_gain * self.p_err + self.i_gain * self.i_err
        return int(np.clip(out, 4, 8))


class Delivery:
    def __init__(self, node):
        self.node = node

        # YOLO 구독자: BoundingBoxes 메시지 받아 callback_yolo 호출
        self.yolo_sub = self.node.create_subscription(
            BoundingBoxes,
            "/darknet_ros/bounding_boxes",
            self.callback_yolo,
            qos_profile_system_default
        )

        # 경로 퍼블리셔
        self.delivery_path_pub = node.create_publisher(
            Path, "/delivery_path", qos_profile_system_default
        )

        # 제어기 초기화
        self.st  = Stanley()
        self.pid = PID(node)
        self.ss  = SpeedSupporter(node)

        # DB 로드 및 원본 경로 읽기
        self.search_path_db   = DB("delivery_search_path.db")
        self.delivery_path_db = DB("delivery_path.db")
        self.search_path      = np.array(self.search_path_db.read_db_n("Path", "x", "y", "yaw"))
        self.delivery_path    = np.array(self.delivery_path_db.read_db_n("Path", "x", "y", "yaw"))

        # 로봇 상태
        self.x = self.y = self.yaw = 0.0

        # 목표 위치: None이면 아직 YOLO 미검출
        self.place_x = None
        self.place_y = None
        # self.place_x = 11.0
        # self.place_y = 15.0

        # 변환된 경로 저장
        self.current_path = None
        self.first_rotation_done = False

        # 퍼블리시 플래그
        self.published_search   = False
        self.published_delivery = False

        # 배달 제어 플래그
        self.first_try         = True
        self.estop_start_time  = None
        self.target_idx        = 0

    def callback_yolo(self, msg):
        """
        YOLO 콜백: abs_var에 해당하는 첫 번째 박스를 찾아 중심좌표 저장
        - 오인식 방지를 위해 confidence(신뢰도)도 사용해서 0.5 이상만 인식하도록 함
        """
        if not self.abs_var:
            return

        best_bb = None
        best_conf = 0.0

        # abs_var에 해당하는 박스 중에서 confidence가 가장 높은 것 선택
        for bb in msg.boxes:
            if bb.class_name == self.abs_var and bb.confidence > 0.5:
                if bb.confidence > best_conf:
                    best_bb = bb
                    best_conf = bb.confidence

        if best_bb:
            self.place_x = (best_bb.xmin + best_bb.xmax) / 2
            self.place_y = (best_bb.ymin + best_bb.ymax) / 2
            # 필요하면 time stamp 등도 같이 기록 가능
        else:
            # abs_var에 해당하는 박스가 없거나 신뢰도가 낮은 경우는 무시
            pass

        

    def rotate_points(self, points: np.ndarray, angle: float, origin: np.ndarray) -> np.ndarray:
        c = m.cos(angle); s = m.sin(angle)
        R = np.array([[c, -s], [s, c]])
        return (points - origin).dot(R.T) + origin

    def db_rotate_adjust(self, path: np.ndarray) -> tuple[list, list, list]:
        """origin → 회전 → 로봇 위치 병진 → yaw 보정"""
        if path is None or len(path) == 0:
            return [], [], []
        ox, oy, oyaw = path[0]
        origin = np.array([ox, oy])
        pts = path[:, :2] - origin
        theta = self.yaw - oyaw
        rotated = self.rotate_points(pts, theta, origin=np.zeros(2))
        final = rotated + np.array([self.x, self.y])
        rel_yaws = path[:,2] - oyaw
        final_yaws = [m.atan2(m.sin(y + self.yaw), m.cos(y + self.yaw)) for y in rel_yaws]
        xs, ys = final[:,0].tolist(), final[:,1].tolist()
        return xs, ys, final_yaws

    def _compute_control(self, path_tuple, odom, estop=0):
        xs, ys, yaws = path_tuple
        steer, self.target_idx, hdr, ctr = self.st.stanley_control(
            odom, xs, ys, yaws, h_gain=0.5, c_gain=0.24
        )
        speed = self.pid.PIDControl(
            odom.v * 3.6,
            self.ss.adaptSpeed(5.0, hdr, ctr, 4, 6)
        )
        msg = ControlMessage()
        msg.steer = int(m.degrees(-steer))
        msg.speed = int(speed)
        msg.gear  = 2
        msg.estop = estop
        return msg

    def publish_path(self, path_tuple):
        xs, ys, yaws = path_tuple
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for x, y, yaw in zip(xs, ys, yaws):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)
        self.delivery_path_pub.publish(path_msg)
        self.node.get_logger().info("경로 퍼블리시 완료.")

    def control_delivery(self, odometry, abs_var):
        #테스트 코드
        # self.place_x, self.place_y= 11.0 , 15.0
        
        # odometry 갱신
        self.x, self.y, self.yaw = odometry.x, odometry.y, odometry.yaw
        self.abs_var = abs_var

        # 첫 회전 변환: yaw가 0이 아닐 때 한 번만
        if not self.first_rotation_done and abs(self.yaw) > 1e-3:
            self.current_path = self.db_rotate_adjust(self.search_path)
            self.first_rotation_done = True

        # 아직 변환 전(yaw==0) 대기
        if self.current_path is None:
            return ControlMessage(), False

        # YOLO 미검출 → 검색 경로
        if self.place_x is None or self.place_y is None:
            if not self.published_search:
                self.publish_path(self.current_path)
                self.published_search = True
            return self._compute_control(self.current_path, odometry), False

        # YOLO 검출 후 거리 계산
        dist = m.hypot(self.place_x - self.x, self.place_y - self.y)

        # 검색 경로 단계
        if dist > 5.0:
            if not self.published_search:
                self.current_path = self.db_rotate_adjust(self.search_path)
                self.publish_path(self.current_path)
                self.published_search = True
            return self._compute_control(self.current_path, odometry), False

        # 배달 경로 단계
        else:
            if self.first_try:
                self.target_idx = 0
                self.first_try = False
            if not self.published_delivery:
                dp = self.db_rotate_adjust(self.delivery_path)
                self.current_path = dp
                self.publish_path(dp)
                self.published_delivery = True

            xs, ys, _ = self.current_path
            remaining = len(xs) - self.target_idx
            now = time.time()

            # 마지막 10개 구간 E-stop
            if remaining <= 10:
                if self.estop_start_time is None:
                    self.estop_start_time = now
                if now - self.estop_start_time < 5.0:
                    return self._compute_control(self.current_path, odometry, estop=1), False
                else:
                    return self._compute_control(self.current_path, odometry, estop=1), True

            return self._compute_control(self.current_path, odometry), False

