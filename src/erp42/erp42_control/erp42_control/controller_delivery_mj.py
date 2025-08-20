#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math as m
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
    - 외부 path(cx, cy, cyaw)를 추종
    - TrafficSign(class_id==abs_var) 감지 후 stop_radius 이내 진입 시:
        E-stop → 5초 정지(HOLD) → 더 빠른 속도로 재추종(RESUME)
    - 경로의 마지막 점에 도달하면: **그 시점의 제어 메시지 그대로** 반환 + done=True
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

        # ── 제어 파라미터 ────────────────────────────────────────────────
        self.v_search = 5.0           # 기본 추종 속도
        self.v_fast = 8.0             # 재출발 후 빠른 추종 속도
        self.max_steer_deg = 28.0     # 조향 제한(deg)
        self.stop_radius = 5.0        # [m] sign까지 거리 임계값
        self.stop_hold_sec = 5.0      # [s] E-stop 유지 시간

        # 경로 완료 판정
        self.finish_radius = 1.0      # [m] 마지막 점 반경
        self.goal_count_thr = 10      # 완료 판정 유지 카운트 (노이즈 억제)

        # 내부
        self.current_path = None      # (xs, ys, yaws)
        self.target_idx = 0
        self.count = 0
        self.published_once = False
        self.goal_count = 0
        self.path_done = False         # 완료 래치

        # ── 상태머신 ────────────────────────────────────────────────────
        # FOLLOW: 평상시 추종
        # HOLD  : E-stop 상태 유지 (stop_hold_sec 동안)
        # RESUME: E-stop 해제, v_fast로 재추종
        self.state = "FOLLOW"
        self._hold_until_sec = None

    # ─────────────────────────────────────────────────────────────────────
    # 콜백 & 유틸
    # ─────────────────────────────────────────────────────────────────────

    def callback_yolo(self, msg: TrafficSign):
        """
        TrafficSign 감지: class_id == abs_var이면 목표 좌표 저장.
        (confidence 필드 사용 시 임계값 체크 추가)
        """
        if self.abs_var is None:
            self.abs_var = 4
        if self.find_sign:
            return
        if msg.class_id != self.abs_var:
            return

        # ⚠️ msg.pose가 map 프레임이라고 가정. 아니라면 TF 변환 필요.
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

        # 조향(단위 변환 및 클램프)
        steer_deg = float(np.degrees(-steer_rad))
        steer_deg = float(np.clip(steer_deg, -self.max_steer_deg, self.max_steer_deg))

        msg = ControlMessage()
        msg.steer = int(steer_deg)         # TODO: ERP42 내부 단위 필요시 변환
        msg.speed = int(speed_cmd * 10.0)  # TODO: 프로젝트 속도 단위에 맞게 스케일
        msg.gear = 2
        msg.estop = int(estop)
        return msg

    def _goal_reached(self, path_tuple) -> bool:
        xs, ys, _ = path_tuple
        if not xs:
            return False
        gx, gy = xs[-1], ys[-1]
        dist = m.hypot(gx - self.x, gy - self.y)
        near_end_idx = (self.target_idx >= max(0, len(xs) - 2))

        if dist <= self.finish_radius and near_end_idx:
            self.goal_count += 1
        else:
            self.goal_count = 0

        return self.goal_count >= self.goal_count_thr

    # ─────────────────────────────────────────────────────────────────────
    # 메인 제어 함수 (상태머신 버전)
    # ─────────────────────────────────────────────────────────────────────

    def control_delivery(self, odometry, abs_var, path):
        """
        외부 path만 추종하며, sign이 잡히고 stop_radius 이내로 접근하면
        E-stop → 5초 정지(HOLD) → v_fast로 재추종(RESUME).
        경로 마지막점 도달 시: **계산한 제어 메시지 그대로 반환** + True.
        - odometry: x, y, yaw 필드를 갖는 객체
        - abs_var : 관심 TrafficSign class_id
        - path    : 외부 제공 경로 객체 (path.cx, path.cy, path.cyaw)
        반환: (ControlMessage, done: bool)
        """
        # 상태 갱신
        self.x, self.y, self.yaw = odometry.x, odometry.y, odometry.yaw
        self.abs_var = abs_var

        # 외부 제공 path → 현재 경로로 사용
        xs = list(path.cx)
        ys = list(path.cy)
        if hasattr(path, "cyaw") and path.cyaw is not None:
            yaws = list(path.cyaw)
        else:
            yaws = [0.0] * len(xs)
        self.current_path = (xs, ys, yaws)

        # 최초 1회 퍼블리시
        if not self.published_once:
            self.publish_path(self.current_path)
            self.published_once = True

        # 현재 시각 (ROS 시간)
        now_sec = self.node.get_clock().now().nanoseconds * 1e-9

        # 만약 이전 호출에서 path_done 래치가 이미 True라면, 계속 원래 메시지 계산해서 True 반환
        if self.path_done:
            speed = self.v_fast if self.state == "RESUME" else self.v_search
            msg = self._compute_control(self.current_path, odometry, speed_cmd=speed, estop=0)
            return msg, True

        # ── 상태 분기 ──────────────────────────────────────────────────
        if self.state == "HOLD":
            # 5초 정지 유지
            if self._hold_until_sec is None:
                self._hold_until_sec = now_sec + self.stop_hold_sec
            if now_sec < self._hold_until_sec:
                return self._safe_stop(), False  # 일시정지 중
            # 정지 유지시간 종료 → 재추종 상태로 전환
            self.state = "RESUME"
            self.find_sign = False       # 같은 표지판으로 재정지 방지
            self._hold_until_sec = None  # 타이머 클리어

        if self.state == "RESUME":
            # 더 빠른 속도로 계속 추종
            msg = self._compute_control(self.current_path, odometry, speed_cmd=self.v_fast)
            if self._goal_reached(self.current_path):
                self.path_done = True
                return msg, True
            return msg, False

        # FOLLOW 상태 (기본)
        if self.find_sign and (self.place_x is not None) and (self.place_y is not None):
            dist = m.hypot(self.place_x - self.x, self.place_y - self.y)

            # 반경 내 진입 시, 약간의 카운트 지연 후 E-stop 래치
            if dist <= self.stop_radius:
                if self.count >= 50:
                    # E-stop 진입 → HOLD 전환 및 타이머 시작
                    self.state = "HOLD"
                    self._hold_until_sec = now_sec + self.stop_hold_sec
                    return self._safe_stop(), False
                else:
                    self.count += 1
                    return self._safe_stop(), False
            else:
                # 반경을 벗어나면 카운트 리셋
                self.count = 0

        # 평상시 (FOLLOW): 기본 속도로 추종
        msg = self._compute_control(self.current_path, odometry, speed_cmd=self.v_search)
        if self._goal_reached(self.current_path):
            self.path_done = True
            return msg, True
        return msg, False
