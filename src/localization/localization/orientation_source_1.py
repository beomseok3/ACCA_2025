import numpy as np


class KalmanFilterYaw:
    def __init__(self):
        # 상태 변수 (yaw 값, 초기값 0)
        self.x = np.array([[0.0]])

        # 상태 전이 행렬 (yaw는 이전 yaw에 변화량이 더해지는 구조)
        self.F = np.array([[1.0]])

        # 제어 입력 행렬 (자이로스코프 yaw 변화량 반영)
        self.B = np.array([[1.0]])

        # 관측 행렬 (GPS yaw 사용, 그대로 측정값 반영)
        self.H = np.array([[1.0]])

        # 초기 오차 공분산 행렬
        self.P = np.array([[1.0]])

        # 프로세스 노이즈 공분산 (IMU 노이즈)
        self.Q = np.array([[0.01]])

        # 관측 노이즈 공분산 (GPS 노이즈)
        self.R = np.array([[0.5]])

    def predict(self, imu_yaw_rate):
        """
        예측 단계: IMU의 yaw 변화량(gyroscope)로 현재 yaw 추정
        """
        u = np.array([[imu_yaw_rate]])
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, gps_yaw):
        """
        업데이트 단계: GPS yaw 값을 사용하여 보정
        """
        z = np.array([[gps_yaw]])
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(1) - K @ self.H) @ self.P

    def get_yaw(self):
        """
        현재 칼만 필터가 추정한 yaw 값 반환
        """
        return self.x[0, 0]


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.qos import qos_profile_sensor_data
from kalman_filter_yaw import KalmanFilterYaw  # 위에서 만든 클래스 import


class Dfilter(Node):
    def __init__(self):
        super().__init__("dfilter_yaw")
        self.create_subscription(
            Imu, "imu/data", self.callback_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "odometry/gps", self.callback_gps, qos_profile_sensor_data
        )
        self.pub_filtered_yaw = self.create_publisher(
            Imu, "imu/filtered_yaw", qos_profile_sensor_data
        )

        self.kf = KalmanFilterYaw()  # 칼만 필터 객체 생성
        self.last_time = self.get_clock().now().nanoseconds / 1e9  # 초기 시간 설정

    def callback_imu(self, msg):
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        # 시간 차이 계산
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # IMU 각속도를 활용하여 yaw 변화량 계산
        imu_yaw_rate = msg.angular_velocity.z * dt
        self.kf.predict(imu_yaw_rate)  # 칼만 필터 예측

        # 보정된 yaw 값 publish
        self.publish_yaw(self.kf.get_yaw())

    def callback_gps(self, msg):
        gps_yaw = (
            msg.pose.pose.orientation.z
        )  # GPS에서 yaw 값 가져오기 ( 미분 혹은 velocity arctan)
        self.kf.update(gps_yaw)  # 칼만 필터 업데이트

    def publish_yaw(self, filtered_yaw):
        msg = Imu()
        quaternion = quaternion_from_euler(0, 0, filtered_yaw)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        self.pub_filtered_yaw.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Dfilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
