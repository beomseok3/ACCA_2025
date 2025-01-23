import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import numpy as np
from DB import DB
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import ControlMessage
from stanley import Stanley
import time


class Delivery:
    def __init__(self, node):
        self.node = node
        self.globalpath_sub = self.node.create_subscription(
            Path,
            "global_path",
            self.callback_global,
            qos_profile=qos_profile_system_default,
        )

        self.delivery_path_pub = self.node.create_publisher(Path, "/delivery_path", 10)

        # Stanley 객체 초기화
        self.st = Stanley()  # Stanley 컨트롤러 인스턴스 생성

        # Lists for storing global path data
        self.gx_list = []
        self.gy_list = []
        self.gyaw_list = []

        # K-city
        # self.abs1_path_db = DB("B1_accurate_acca.db")
        # self.abs2_path_db = DB("B2_accurate_acca.db")
        # self.abs3_path_db = DB("B3_accurate_acca.db")

        # dolge
        self.abs1_path_db = DB("B1_dolge_test.db")
        self.abs2_path_db = DB("B2_dolge_test.db")
        self.abs3_path_db = DB("B3_dolge_test.db")

        # 데이터베이스에서 모든 경로를 한 번에 읽어옴
        self.abs1_path = np.array(self.abs1_path_db.read_db_n("Path", "x", "y", "yaw"))
        self.abs2_path = np.array(self.abs2_path_db.read_db_n("Path", "x", "y", "yaw"))
        self.abs3_path = np.array(self.abs3_path_db.read_db_n("Path", "x", "y", "yaw"))

        # 경로 선택 (abs1, abs2, abs3 중 하나 선택)
        self.abs_var = "abs3"  # "abs1" or "abs2" or "abs3"
        self.db = self.select_path(self.abs_var)

        self.estop = 0
        self.target_idx = 0
        self.delivery_finished = False
        self.path_published_once = False  # 경로가 한 번만 publish 되도록 설정

        # 새로운 경로 추가를 위한 변수 초기화
        self.speed = 0
        # self.additional_path_published = False
        # self.set_additional_path_endpoints()  # 각 abs_var에 맞는 종료 좌표 설정
        self.count = 0  # estop 유지 카운터 초기화

    # def set_additional_path_endpoints(self):
    #     """abs_var에 따른 종료 지점 좌표를 설정"""
    #     if self.abs_var == "abs1":
    #         self.additional_path_x = 16337  # abs1의 종료 x 좌표
    #         self.additional_path_y = 28473.6  # abs1의 종료 y 좌표
    #     elif self.abs_var == "abs2":
    #         self.additional_path_x = 16338.7  # abs2의 종료 x 좌표
    #         self.additional_path_y = 28468  # abs2의 종료 y 좌표
    #     elif self.abs_var == "abs3":
    #         self.additional_path_x = 16339.2  # abs3의 종료 x 좌표
    #         self.additional_path_y = 28462.5  # abs3의 종료 y 좌표

    #     else:
    #         raise ValueError(f"Invalid abs_var: {self.abs_var}")

    def select_path(self, abs_var):
        """abs_var에 따라 선택된 경로를 반환"""
        if abs_var == "abs1":
            return self.abs1_path
        elif abs_var == "abs2":
            return self.abs2_path
        elif abs_var == "abs3":
            return self.abs3_path
        else:
            raise ValueError(f"Invalid abs_var: {abs_var}")

    def callback_global(self, msg):
        for p in msg.poses:
            self.gx_list.append(p.pose.position.x)
            self.gy_list.append(p.pose.position.y)
            _, _, yaw = euler_from_quaternion(
                [
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ]
            )
            self.gyaw_list.append(yaw)

    def control_delivery(self, odometry):
        """Delivery 경로에 따른 제어 로직."""
        msg = ControlMessage()

        if not self.path_published_once:
            self.publish_path()  # 경로를 한 번만 publish

        if len(self.db) != 0:  # 경로가 있는 경우
            # 경로 데이터 가져오기
            path_cx, path_cy, path_cyaw = self.db[:, 0], self.db[:, 1], self.db[:, 2]

            # Stanley 제어 수행
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry, path_cx, path_cy, path_cyaw, h_gain=0.5, c_gain=0.24
            )

            # 경로의 마지막에 도달했는지 확인
            if self.target_idx >= len(path_cx) - 1:
                print(f"Target index reached: {self.target_idx}/{len(path_cx)}")

                # estop 활성화 및 50회 동안 유지
                if self.count < 50:
                    self.estop = 1  # estop 유지
                    self.count += 1  # 카운트 증가
                    self.node.get_logger().info(f"Stopping... count: {self.count}")
                else:
                    # 50회 이후 estop 해제 및 완료 표시
                    self.estop = 0
                    self.delivery_finished = True  # delivery 완료 표시
                    self.node.get_logger().info("Delivery finished.")

            # 속도 설정 (경로가 끝에 도달하지 않았을 때)
            if not self.delivery_finished:
                self.speed = 3  
            else:
                self.speed = 0  # 완료 시 속도 0

        else:
            print("not delivery path")
            # 경로가 없을 때도 Stanley 제어 수행
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry, path_cx, path_cy, path_cyaw, h_gain=0.5, c_gain=0.24
            )
            self.estop = 0  # 경로가 없을 때는 estop 해제

        # Control 메시지 생성
        msg.steer = int(math.degrees(-steer))  # 조향 각도 설정
        msg.speed = self.speed * 10  # 속도 설정 (기본 속도 * 10)
        msg.gear = 2  # 전진 기어 설정
        msg.estop = self.estop  # estop 상태 설정

        return msg, self.delivery_finished  # Control 메시지와 완료 여부 반환


    def publish_path(self):
        if self.path_published_once:
            return

        path_data = self.db

        if path_data is not None and len(path_data) > 0:
            path_msg = Path()
            path_msg.header.stamp = self.node.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"

            for x, y, yaw in path_data:
                pose = PoseStamped()
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path_msg.poses.append(pose)

            self.delivery_path_pub.publish(path_msg)
            self.path_published_once = True
            print("Path published successfully.")
        else:
            self.node.get_logger().error("Failed to load path from database")
            print("Error: No path found in the database.")

    # def publish_additional_path(self, odometry):
    #     """목표 도달 시 추가로 생성된 경로를 Stanley control과 함께 publish"""
    #     path_msg = Path()
    #     path_msg.header.stamp = self.node.get_clock().now().to_msg()
    #     path_msg.header.frame_id = "map"

    #     # 추가 경로 생성 (직선 경로)
    #     last_x = self.db[-1, 0]  # 현재 경로의 마지막 x 좌표
    #     last_y = self.db[-1, 1]  # 현재 경로의 마지막 y 좌표

    #     # 직선 경로의 총 거리 계산
    #     total_distance = math.sqrt(
    #         (self.additional_path_x - last_x) ** 2
    #         + (self.additional_path_y - last_y) ** 2
    #     )

    #     # 0.1m 간격으로 추가 경로의 포즈 개수 계산
    #     self.num_poses = int(total_distance / 0.1)

    #     # 시작점에서 끝점으로의 yaw 계산
    #     direction_yaw = math.atan2(
    #         self.additional_path_y - last_y, self.additional_path_x - last_x
    #     )

    #     # 추가 경로 포즈 생성
    #     path_cx = []
    #     path_cy = []
    #     path_cyaw = []
    #     for i in range(self.num_poses):
    #         pose = PoseStamped()
    #         pose.header.stamp = self.node.get_clock().now().to_msg()
    #         pose.header.frame_id = "map"

    #         # 각 포즈의 좌표를 직선 경로로 배치
    #         x = last_x + i * (self.additional_path_x - last_x) / self.num_poses
    #         y = last_y + i * (self.additional_path_y - last_y) / self.num_poses
    #         pose.pose.position.x = x
    #         pose.pose.position.y = y
    #         pose.pose.position.z = 0.0

    #         # 각 포즈의 orientation을 경로의 시작점에서 끝점 방향으로 설정
    #         quaternion = quaternion_from_euler(0, 0, direction_yaw)
    #         pose.pose.orientation.x = quaternion[0]
    #         pose.pose.orientation.y = quaternion[1]
    #         pose.pose.orientation.z = quaternion[2]
    #         pose.pose.orientation.w = quaternion[3]
    #         path_msg.poses.append(pose)

    #         path_cx.append(x)
    #         path_cy.append(y)
    #         path_cyaw.append(direction_yaw)

    #     # 추가 경로 publish
    #     self.delivery_path_pub.publish(path_msg)
    #     print(f"Additional path for {self.abs_var} published.")

    #     # Stanley control을 사용해 steer와 target_idx 계산
    #     steer, self.target_idx, hdr, ctr = self.st.stanley_control(
    #         odometry,
    #         path_cx,
    #         path_cy,
    #         path_cyaw,
    #         h_gain=0.5,
    #         c_gain=0.24,
    #     )

    #     print("target_idx of delivery-return_path: ", self.target_idx)

    #     # Control 메시지 생성
    #     msg = ControlMessage()
    #     msg.steer = int(math.degrees((-1) * steer))
    #     msg.speed = self.speed * 10
    #     msg.gear = 2
    #     msg.estop = self.estop

    #     # 추가 경로의 마지막 idx에 도달했는지 확인
    #     if self.target_idx >= len(path_cx) - 5:
    #         self.delivery_finished = True  # 추가 경로의 마지막에 도달하면 종료
    #         self.node.get_logger().info("Reached the end of the additional path.")
