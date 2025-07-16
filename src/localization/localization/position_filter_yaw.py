import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from erp42_msgs.msg import SerialFeedBack
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class Pfilter(Node):
    def __init__(self):
        super().__init__("pfilter")

        self.create_subscription(
            SerialFeedBack, "erp42_feedback", self.is_dynamic, qos_profile_sensor_data
        )

        self.create_subscription(
            Imu, "imu/data", self.pfilter, qos_profile_sensor_data
        )

        self.filter_pub = self.create_publisher(
            Imu, "imu/pfiltered", qos_profile_sensor_data
        )

        # Heading 값 저장 변수
        self.last_heading = None  # 정적 상태에서의 yaw 값
        self.is_moving = False  # 동적인지 여부 저장
        self.yaw_drift = 0.0  # 정적 상태에서 누적된 드리프트 값

    def is_dynamic(self, msg):
        self.is_moving = msg.speed > 0 # or msg.steer != 0

    def pfilter(self, msg):
        # 쿼터니언 -> 오일러 변환
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(q)

        if not self.is_moving:  # 정적 상태일 때
            if self.last_heading is None:
                self.last_heading = yaw  # 첫 번째 값 저장

            self.yaw_drift = yaw - self.last_heading  # 드리프트를 누적해서 저장
            yaw = self.last_heading  # yaw 변화를 강제로 0으로 만듦

        else:  # 동적 상태일 때
            yaw -= self.yaw_drift  # 정적 상태에서 쌓인 드리프트만 보정
            self.last_heading = yaw  # 현재 yaw 값을 저장

        # 보정된 yaw 값을 다시 쿼터니언으로 변환
        new_q = quaternion_from_euler(roll, pitch, yaw)

        # 새로운 IMU 메시지 생성
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.orientation.x = new_q[0]
        filtered_msg.orientation.y = new_q[1]
        filtered_msg.orientation.z = new_q[2]
        filtered_msg.orientation.w = new_q[3]
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.linear_acceleration = msg.linear_acceleration

        self.filter_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Pfilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
