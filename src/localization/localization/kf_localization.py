#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import math


class ExtendedKalmanFilter:
    def __init__(self):
        # 상태: [x, y, theta] (미터, 미터, 라디안)
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1

        # 프로세스 노이즈 공분산 (모델에 따라 튜닝)
        self.Q = np.diag([0.1, 0.1, np.deg2rad(5) ** 2])
        # 측정 노이즈 공분산 (GPS 위치 측정 노이즈)
        self.R = np.diag([5.0, 5.0])

    def predict(self, u, dt):
        """
        예측 단계:
          u: [v, omega] (전진 속도 (m/s), 각속도 (rad/s))
          dt: 시간 간격 (s)
        """
        v, omega = u
        x, y, theta = self.x.flatten()

        # 비선형 상태 전이 (자전거 모델 형태, 회전이 0에 가까운 경우도 처리)
        if abs(omega) < 1e-6:
            x_new = x + v * dt * math.cos(theta)
            y_new = y + v * dt * math.sin(theta)
            theta_new = theta
        else:
            x_new = x + (v / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
            y_new = y - (v / omega) * (math.cos(theta + omega * dt) - math.cos(theta))
            theta_new = theta + omega * dt

        self.x = np.array([[x_new], [y_new], [theta_new]])

        # 상태 전이 행렬 F (Jacobian wrt state)
        F = np.eye(3)
        if abs(omega) < 1e-6:
            F[0, 2] = -v * dt * math.sin(theta)
            F[1, 2] = v * dt * math.cos(theta)
        else:
            F[0, 2] = (v / omega) * (math.cos(theta + omega * dt) - math.cos(theta))
            F[1, 2] = (v / omega) * (math.sin(theta + omega * dt) - math.sin(theta))

        # 예측 오차 공분산 업데이트
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        """
        업데이트 단계:
          z: 측정값, 여기서는 [x, y] (GPS 위치, 단위: m)
        """
        # 측정 모델: h(x) = [x, y]
        H = np.array([[1, 0, 0], [0, 1, 0]])
        z = np.array(z).reshape((2, 1))
        y_residual = z - H @ self.x  # innovation

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y_residual
        self.P = (np.eye(3) - K @ H) @ self.P


class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__("ekf_localization")
        self.ekf = ExtendedKalmanFilter()
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # 구독: IMU, 엔코더(Odometry), GPS (NavSatFix)
        self.create_subscription(Imu, "imu/data", self.imu_callback, 10)
        self.create_subscription(Odometry, "encoder/odom", self.encoder_callback, 10)
        self.create_subscription(NavSatFix, "gps/fix", self.gps_callback, 10)

        # EKF 결과 publish (Odometry 메시지)
        self.odom_pub = self.create_publisher(Odometry, "ekf/odom", 10)

        # 제어 입력: [v, omega]
        self.current_control = np.array([0.0, 0.0])

    def imu_callback(self, msg):
        # IMU에서 각속도 추출 (z축, rad/s)
        omega = msg.angular_velocity.z
        self.current_control[1] = omega
        self.predict_step()

    def encoder_callback(self, msg):
        # 엔코더로부터 전진 속도(v) 추출 (Odometry 메시지의 twist)
        v = msg.twist.twist.linear.x
        self.current_control[0] = v
        self.predict_step()

    def gps_callback(self, msg):
        # GPS 측정값 업데이트
        # 실제로는 위도/경도를 변환하는 과정이 필요함.
        # 예제에서는 간단히 msg.latitude, msg.longitude를 x, y로 사용 (단위: m)
        x = msg.latitude  # 예시용, 실제 변환 필요
        y = msg.longitude  # 예시용, 실제 변환 필요
        self.ekf.update([x, y])
        self.publish_odom()

    def predict_step(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # 예측 단계: 현재 제어 입력과 dt 사용
        self.ekf.predict(self.current_control, dt)
        self.publish_odom()

    def publish_odom(self):
        # EKF 결과를 Odometry 메시지로 publish
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        odom_msg.pose.pose.position.x = self.ekf.x[0, 0]
        odom_msg.pose.pose.position.y = self.ekf.x[1, 0]
        # orientation: theta -> quaternion
        theta = self.ekf.x[2, 0]
        q = self.euler_to_quaternion(0, 0, theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
