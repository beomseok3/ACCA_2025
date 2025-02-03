import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf_transformations import *
import math as m
import numpy as np
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class IMU(Node):
    def __init__(self):
        super().__init__("imu_odometry")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom_imu")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        imu_topic = self.get_parameter("imu_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile=qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(
            Odometry, odom_topic, qos_profile=qos_profile_system_default
        )

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.init_yaw = 0.0
        self.v = 0.0
        self.last_time = None

    # def imu_callback(self, msg):
    #     q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    #     euler = euler_from_quaternion(q)
    #     self.yaw = euler[2]

    #     current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    #     dt = (current_time - self.last_time) if self.last_time else 0.0
    #     self.last_time = current_time

    #     self.x += self.v * np.cos(self.yaw) * dt
    #     self.y += self.v * np.sin(self.yaw) * dt

    #     a_x = msg.linear_acceleration.x

    #     self.v += a_x * dt

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = euler_from_quaternion(q)

        if self.init_yaw == 0.0:
            self.init_yaw = euler[2]
            # return

        self.yaw = euler[2] - self.init_yaw
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time
            # return

        dt = current_time - self.last_time
        self.last_time = current_time

        # a_x = msg.linear_acceleration.x * np.cos(-self.yaw) + msg.linear_acceleration.y * np.sin(-self.yaw)
        # a_y = -msg.linear_acceleration.x * np.sin(-self.yaw) + msg.linear_acceleration.y * np.cos(-self.yaw)

        a_x = msg.linear_acceleration.y * np.cos(
            self.yaw
        ) - msg.linear_acceleration.x * np.sin(self.yaw)
        # a_y = msg.linear_acceleration.x * np.sin(self.yaw) + msg.linear_acceleration.y * np.cos(self.yaw)

        self.v += a_x * dt
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(msg)

    # def imu_callback(self, msg):
    #     # Orientation -> Euler angles 변환
    #     q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    #     euler = euler_from_quaternion(q)
    #     self.yaw = euler[2]  # Yaw 값을 업데이트

    #     # 시간 계산
    #     current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    #     if self.last_time is None:
    #         self.last_time = current_time
    #         return

    #     dt = current_time - self.last_time
    #     self.last_time = current_time

    #     # 센서 프레임 -> 월드 좌표계 변환
    #     a_x = msg.linear_acceleration.x
    #     a_y = msg.linear_acceleration.y

    #     # 월드 좌표계에서의 가속도
    #     a_world_x = a_x * np.cos(self.yaw) - a_y * np.sin(self.yaw)
    #     a_world_y = a_x * np.sin(self.yaw) + a_y * np.cos(self.yaw)

    #     # 속도와 위치 업데이트
    #     self.v_x += a_world_x * dt
    #     self.v_y += a_world_y * dt

    #     self.x += self.v_x * dt
    #     self.y += self.v_y * dt


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
