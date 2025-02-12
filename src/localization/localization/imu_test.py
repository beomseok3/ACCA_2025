import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from tf_transformations import *

import numpy as np


class Imu_test(Node):
    def __init__(self):
        super().__init__("imu_test")

        # sub
        self.create_subscription(
            Imu, "imu/data", self.callback_imu, qos_profile_sensor_data
        )

        # pub
        self.pub_deg_yaw = self.create_publisher(
            Float32, "imu/yaw", qos_profile_system_default
        )

        self.prev_r, self.prev_p, self.prev_y = None, None, None

    def callback_imu(self, msg):

        r, p, y = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )

        r = np.rad2deg(r)
        p = np.rad2deg(p)
        y = np.rad2deg(y)

        if self.prev_r is not None:
            self.get_logger().info(
                f"r: {r - self.prev_r}, p: {p - self.prev_p}, y: {y - self.prev_y}"
            )
            self.get_logger().warn(f"r: {r}, p: {p}, y: {y}")

        self.prev_r = r
        self.prev_p = p
        self.prev_y = y

        self.pub_deg_yaw.publish(Float32(data=y))


def main(args=None):
    rclpy.init(args=args)
    node = Imu_test()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
