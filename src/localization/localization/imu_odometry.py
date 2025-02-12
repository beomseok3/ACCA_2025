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
        self.declare_parameter("imu_gps_topic", "/imu/rotated")
        self.declare_parameter("odom_imu_topic", "/odom_imu")
        self.declare_parameter("odom_imu_gps_topic", "/odom_imu_gps")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        imu_topic = self.get_parameter("imu_topic").value
        imu_gps_topic = self.get_parameter("imu_gps_topic").value
        odom_imu_topic = self.get_parameter("odom_imu_topic").value
        odom_imu_gps_topic = self.get_parameter("odom_imu_gps_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # subscriber
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile=qos_profile_sensor_data
        )
        self.imu_gps_sub = self.create_subscription(
            Imu, imu_gps_topic, self.imu_gps_callback, qos_profile=qos_profile_sensor_data
        )

        #publisher
        self.odom_imu_pub = self.create_publisher(
            Odometry, odom_imu_topic, qos_profile=qos_profile_system_default
        )
        self.odom_imu_gps_pub = self.create_publisher(
            Odometry, odom_imu_gps_topic, qos_profile=qos_profile_system_default
        )


        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.yaw_ = 0.0
        self.yaw_gps = 0.0
        self.init_yaw = None
        self.init_yaw_gps = None

        self.last_time = None

    def imu_gps_callback(self,msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = euler_from_quaternion(q)

        # if self.init_yaw_gps is None:
        #     self.init_yaw_gps = euler[2]
        self.init_yaw_gps = 0.0
        self.yaw_gps = euler[2] - self.init_yaw_gps

        msg__ = Odometry()
        msg__.header.stamp = self.get_clock().now().to_msg()
        msg__.header.frame_id = self.frame_id
        msg__.child_frame_id = self.child_frame_id
        msg__.pose.pose.position.x = self.x
        msg__.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.yaw_gps)
        msg__.pose.pose.orientation.x = q[0]
        msg__.pose.pose.orientation.y = q[1]
        msg__.pose.pose.orientation.z = q[2]
        msg__.pose.pose.orientation.w = q[3]
        self.odom_imu_gps_pub.publish(msg__)

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = euler_from_quaternion(q)

<<<<<<< HEAD
        # if self.init_yaw == 0.0:
        #     self.init_yaw = euler[2]
        #     # return

        # self.yaw = euler[2] - self.init_yaw
        self.yaw = euler[2]
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
=======
        # if self.init_yaw is None:
        #     self.init_yaw = euler[2]
        #     print(self.init_yaw)
        self.init_yaw = 0.
        self.yaw_ = euler[2] - self.init_yaw
>>>>>>> 905bcc9 (commit_com)

        msg__ = Odometry()
        msg__.header.stamp = self.get_clock().now().to_msg()
        msg__.header.frame_id = self.frame_id
        msg__.child_frame_id = self.child_frame_id
        msg__.pose.pose.position.x = self.x
        msg__.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.yaw_)
        msg__.pose.pose.orientation.x = q[0]
        msg__.pose.pose.orientation.y = q[1]
        msg__.pose.pose.orientation.z = q[2]
        msg__.pose.pose.orientation.w = q[3]
        self.odom_imu_pub.publish(msg__)

def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
