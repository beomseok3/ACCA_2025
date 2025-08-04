#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math as m
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler

from yolo_msg.msg import TrafficSign          # class_id, pose, (optional) confidence
from erp42_msgs.msg import ControlMessage
from stanley import Stanley                   # stanley_control(odom, xs, ys, yaws, ...)-> (steer_rad, target_idx, hdr, ctr)


class Delivery:
    """
    - 외부에서 들어오는 path(cx, cy, cyaw)를 항상 추종
    - TrafficSign으로 목표(self.place_x, self.place_y) 감지 후,
      현재 위치와의 거리가 stop_radius(기본 1.0 m) 이하가 되는 즉시 e-stop
    """

    def __init__(self, node: Node):
        self.node = node

        # YOLO 구독자
        self.yolo_sub = self.node.create_subscription(
            TrafficSign, "/traffic_sign_map", self.callback_yolo, 10
        )

        # 경로 퍼블리셔(시각화용)
        self.delivery_path_pub = self.node.create_publisher(Path, "/delivery_path", 10)

        # 제어기
        self.st = Stanley()

        # 로봇 상태
        self.x = self.y = self.yaw = 0.0

        # TrafficSign 목표
        self.place_x = None
        self.place_y = None
        self.find_sign = False
        self.abs_var = None  # 관심 class_id (외부에서 전달 받음)

        # 제어 파라미터
        self.v_search = 5.0          # 추종 속도(프로젝트 스펙 단위에 맞게 조정)
        self.max_steer_deg = 28.0     # 조향 제한(deg)
        self.stop_radius = 1.0        # [m] sign까지 거리 임계값 (1 m 내면 즉시 정지)

        # 내부
        self.current_path = None      # (xs, ys, yaws)
        self.target_idx = 0
        self.count = 0
        self.published_once = False

    # ─────────────────────────────────────────────────────────────────────
    # 콜백 & 유틸
    # ─────────────────────────────────────────────────────────────────────

    def callback_yolo(self, msg: TrafficSign):
        """
        TrafficSign 감지: class_id == abs_var이면 목표 좌표 저장.
        (confidence 필드가 있으면 임계값을 추가하세요.)
        """
        if self.abs_var is None:
            # 기본값이 필요하다면 지정 (예: 4)
            self.abs_var = 4
        if self.find_sign:
            return
        if msg.class_id != self.abs_var:
            return

        # ⚠️ msg.pose가 map 프레임이라고 가정. 아니라면 TF 변환 추가 필요.
        self.place_x = msg.pose.position.x
        self.place_y = msg.pose.position.y
        self.find_sign = True

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

    def _safe_stop(self) -> ControlMessage:
        msg = ControlMessage()
        msg.steer = 0
        msg.speed = 0
        msg.gear = 2
        msg.estop = 1
        return msg

    def _compute_control(self, path_tuple, odom, speed_cmd: float, estop: int = 0) -> ControlMessage:
        xs, ys, yaws = path_tuple
        steer_rad, self.target_idx, hdr, ctr = self.st.stanley_control(
            odom, xs, ys, yaws, h_gain=0.5, c_gain=0.24
        )

        # 조향(단위 변환 및 클램프) - 프로젝트 스펙에 맞춰 수정하세요.
        steer_deg = float(np.degrees(-steer_rad))
        steer_deg = float(np.clip(steer_deg, -self.max_steer_deg, self.max_steer_deg))

        msg = ControlMessage()
        msg.steer = int(steer_deg)         # TODO: ERP42 내부 단위 필요시 변환
        msg.speed = int(speed_cmd * 10.0)  # TODO: 프로젝트 속도 단위에 맞게 스케일
        msg.gear = 2
        msg.estop = int(estop)
        return msg

    # ─────────────────────────────────────────────────────────────────────
    # 메인 제어 함수 (단순화 버전)
    # ─────────────────────────────────────────────────────────────────────

    def control_delivery(self, odometry, abs_var, path):
        """
        외부 path만 추종하며, sign이 잡히고 1 m 이내로 접근하면 즉시 e-stop.
        - odometry: x, y, yaw 필드를 갖는 객체
        - abs_var : 관심 TrafficSign class_id
        - path    : 외부 제공 경로 객체 (path.cx, path.cy, path.cyaw)
        반환: (ControlMessage, done: bool)
        """
        # 상태 갱신
        self.x, self.y, self.yaw = odometry.x, odometry.y, odometry.yaw
        self.abs_var = abs_var

        # 1) 외부 제공 path → 현재 경로로 사용
        xs = list(path.cx)
        ys = list(path.cy)
        if hasattr(path, "cyaw") and path.cyaw is not None:
            yaws = list(path.cyaw)
        else:
            # 길이 맞춤
            yaws = [0.0] * len(xs)
        self.current_path = (xs, ys, yaws)

        # 최초 1회 퍼블리시
        if not self.published_once:
            self.publish_path(self.current_path)
            self.published_once = True

        # 2) sign이 잡혔고, 목표 좌표가 있으며, 1 m 이내면 즉시 e-stop
        if self.find_sign and (self.place_x is not None) and (self.place_y is not None):
            dist = m.hypot(self.place_x - self.x, self.place_y - self.y)
            if dist <= self.stop_radius and self.count >= 50:
                return self._safe_stop(), True
            elif dist <= self.stop_radius:
                self.count += 1
                return self._safe_stop(), False

        # 3) 그 외에는 계속 추종
        return self._compute_control(self.current_path, odometry, speed_cmd=self.v_search), False
