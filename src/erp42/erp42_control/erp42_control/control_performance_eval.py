import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

import math as m
import threading
import time


class Pose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.h = 0.0  # heading (yaw)


def normalize_angle(angle):
    # 각도를 [-pi, pi] 범위로 정규화
    return m.atan2(m.sin(angle), m.cos(angle))


class Eval_control(Node):
    def __init__(self):
        super().__init__("eval_control")

        # subscribe to vehicle's localization (Odometry) and reference path (Marker)
        self.sub_local = self.create_subscription(
            Odometry,
            "localization/kinematic_state",
            self.callback_local,
            qos_profile_sensor_data,
        )

        self.sub_path = self.create_subscription(
            Marker, "ref_traj_marker", self.callback_ref_path, qos_profile_sensor_data
        )

        # 누적 RMSE 계산을 위한 변수 초기화
        self.heading_error_squared_sum = 0.0
        self.lateral_error_squared_sum = 0.0
        self.sample_count = 0

        # 현재 pose 변수 (초기에는 None)
        self.local_pose = None
        self.path_pose = None

        # 별도의 쓰레드에서 RMSE를 누적해서 계산하는 함수 실행
        self.rmse_thread = threading.Thread(target=self.another_thread_rmse)
        self.rmse_thread.daemon = True
        self.rmse_thread.start()

    def callback_local(self, msg):
        self.local_pose = Pose()
        self.local_pose.x = msg.pose.pose.position.x
        self.local_pose.y = msg.pose.pose.position.y
        # orientation 필드의 오타 수정: orientation
        _, _, yaw = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        self.local_pose.h = yaw

    def callback_ref_path(self, msg):
        # 경로의 첫 두 점을 이용하여 path의 pose를 정의 (적어도 2개의 점 필요)
        if len(msg.points) < 2:
            return  # 충분한 데이터가 없으면 무시
        first_point = msg.points[0]
        second_point = msg.points[1]
        self.path_pose = Pose()
        self.path_pose.x = first_point.x
        self.path_pose.y = first_point.y
        yaw = m.atan2(second_point.y - first_point.y, second_point.x - first_point.x)
        self.path_pose.h = yaw

    def another_thread_rmse(self):
        """
        이 함수는 별도의 쓰레드에서 1초마다 현재까지의 heading error와 lateral error의
        누적 제곱합을 이용하여 RMSE를 계산합니다.
        """
        rate = 0.1  # 1초 주기
        while rclpy.ok():
            # 양쪽 pose가 모두 업데이트된 경우에만 계산
            if self.local_pose is None or self.path_pose is None:
                time.sleep(rate)
                continue

            # Heading error: 차량 yaw와 경로 yaw의 차이 (각도 정규화 적용)
            heading_error = normalize_angle(self.local_pose.h - self.path_pose.h)

            # Lateral error 계산:
            # dx, dy는 차량과 경로의 위치 차이이고, 경로의 heading(진행방향)에 수직인 방향으로 투영
            dx = self.local_pose.x - self.path_pose.x
            dy = self.local_pose.y - self.path_pose.y
            lateral_error = -m.sin(self.path_pose.h) * dx + m.cos(self.path_pose.h) * dy

            # 누적 제곱 오차 업데이트
            self.heading_error_squared_sum += heading_error**2
            self.lateral_error_squared_sum += lateral_error**2
            self.sample_count += 1

            # 현재까지의 RMSE 계산
            heading_rmse = m.sqrt(self.heading_error_squared_sum / self.sample_count)
            lateral_rmse = m.sqrt(self.lateral_error_squared_sum / self.sample_count)

            # heading_rmse = m.degrees(heading_rmse) rad -> deg
            self.get_logger().info(
                f"Heading RMSE: {heading_rmse:.3f}, Lateral RMSE: {lateral_rmse:.3f}"
            )

            time.sleep(rate)


def main(args=None):
    rclpy.init(args=args)
    node = Eval_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
