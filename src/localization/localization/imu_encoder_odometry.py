import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


class Imu_Encoder(Node):
    def __init__(self):
        super().__init__("imu_encoder_odometry")

        self.declare_parameter("imu_topic", "/imu/rotated")
        self.declare_parameter("odom_topic", "/odom_imu_encoder")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        imu_topic = self.get_parameter("imu_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # subscriber
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile=qos_profile_sensor_data
        )
        self.erp_twist_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            "erp42/twist",
            self.callback_erp_twist,
            qos_profile=qos_profile_sensor_data,
        )

        # publisher(odometry)
        self.odom_pub = self.create_publisher(
            Odometry, odom_topic, qos_profile=qos_profile_system_default
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.init_yaw = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.last_time = None

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = euler_from_quaternion(q)

        # if self.init_yaw == 0.0:
        #     self.init_yaw = euler[2]

        # self.yaw = euler[2] - self.init_yaw
        self.yaw = euler[2]

    def callback_erp_twist(self, msg):
        print("ongoing")
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time

        dt = current_time - self.last_time
        self.last_time = current_time

        self.v_x = msg.twist.twist.linear.x
        self.v_y = msg.twist.twist.linear.y
        print(f"{self.v_x}  {self.v_y}")

        self.x = self.x + float(self.v_x) * dt
        self.y = self.y + float(self.v_y) * dt
        print(f"{self.x}    {self.y}    {dt}")

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


def main(args=None):
    rclpy.init(args=args)
    imu_encoder_node = Imu_Encoder()
    rclpy.spin(imu_encoder_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
