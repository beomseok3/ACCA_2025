import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import *
import math as m
import numpy as np
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class Dfilter(Node):
    def __init__(self):
        super().__init__("dfilter_yaw")
        self.create_subscription(
            Imu, "imu/data", self.callback_imu, qos_profile_system_default
        )

        self.pub_filtered_yaw = self.create_publisher(
            Imu, "imu/data_dfiltered", qos_profile_system_default
        )

        self.prev_value = 0.0
        self.value = 0.0

    def dfilter(self, value):
        dvalue = value - self.prev_value

        if self.value == 0.0:
            self.value = value
            self.prev_value = self.value
            return self.value

        if 0.015 > abs(dvalue):
            self.value += dvalue
        self.prev_value = value

        return self.value

    def callback_imu(self, msg):
        print("hello")
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        euler = euler_from_quaternion(quaternion)
        yaw = self.dfilter(euler[2])
        quaternion = quaternion_from_euler(0, 0, yaw)
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
