import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import *
from std_msgs.msg import String, Float64
from rclpy.qos import QoSProfile, qos_profile_system_default, qos_profile_sensor_data
import math as m
import numpy as np
from sensor_msgs.msg import Imu


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


class Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Orientation:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Gps_velocity(Node):
    def __init__(self):
        super().__init__("fix_velocity")
        # self.create_subscription(
        #     Float64, "gps/yaw", self.callback_gps, qos_profile_system_default
        # )
        # self.create_subscription(
        #     Imu, "imu/rotated", self.callback_gps, qos_profile_system_default
        # )

        self.create_subscription(
            Quaternion, "mean", self.callback_gps, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "odometry/navsat", self.callback_odom, qos_profile_system_default
        )

        self.pub_odom = self.create_publisher(
            Odometry, "odometry/velocity", qos_profile_system_default
        )
        self.position = Position(0.0, 0.0, 0.0)
        self.gps_yaw = 0.0

    def callback_gps(self, msg):
        # yaw = msg.
        # Adjusting yaw to match the expected orientation

        # self.gps_yaw = normalize_angle(math.pi / 2.0 - math.atan2(vy, vx))

        # quat = quaternion_from_euler(0, 0, yaw)
        # print(quat)
        msg_ = Odometry()
        msg_.header.stamp = self.get_clock().now().to_msg()
        msg_.header.frame_id = "odom"
        # msg_.pose.pose.orientation.x = quat[0]
        # msg_.pose.pose.orientation.y = quat[1]
        # msg_.pose.pose.orientation.z = quat[2]
        # msg_.pose.pose.orientation.w = quat[3]
        # msg_.pose.pose.orientation.x = msg.orientation.x
        # msg_.pose.pose.orientation.y = msg.orientation.y
        # msg_.pose.pose.orientation.z = msg.orientation.z
        # msg_.pose.pose.orientation.w = msg.orientation.w
        # msg_.pose.pose.orientation = Quaternion(
        #     x=self.orientation.x,
        #     y=self.orientation.y,
        #     z=self.orientation.z,
        #     w=self.orientation.w,
        # )
        msg_.pose.pose.orientation = Quaternion(
            x=msg.x,
            y=msg.y,
            z=msg.z,
            w=msg.w,
        )
        msg_.pose.pose.position.x = self.position.x
        msg_.pose.pose.position.y = self.position.y
        msg_.pose.pose.position.z = self.position.z
        self.pub_odom.publish(msg_)

    def callback_odom(self, msg):

        self.position = Position(
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        )
        self.orientation = Orientation(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Gps_velocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
