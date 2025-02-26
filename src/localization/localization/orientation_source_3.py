import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import math


class GyroHeadingNode(Node):
    def __init__(self):
        super().__init__("gyro_heading_node")
        self.subscription = self.create_subscription(
            Imu, "imu/data", self.imu_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.heading = 0.0  # 초기 heading (라디안)
        self.last_time = self.get_clock().now().nanoseconds / 1e9  # 초기 시간

    def imu_callback(self, msg):
        # 자이로 각속도 (angular_velocity.z) 사용, rad/s 단위
        gyro_z = msg.angular_velocity.z

        # 현재 시간 (초 단위)
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # 각속도를 시간에 대해 적분하여 heading 업데이트
        self.heading += gyro_z * dt

        # heading 값이 -pi ~ pi 범위를 벗어나지 않도록 정규화 (옵션)
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

        # heading을 도(degree) 단위로 변환하여 출력
        heading_deg = self.heading * 180.0 / math.pi
        self.get_logger().info(f"Calculated Heading: {heading_deg:.2f} degrees")


def main(args=None):
    rclpy.init(args=args)
    node = GyroHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
