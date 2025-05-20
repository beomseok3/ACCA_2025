import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np


class ImuEncoder(Node):
    def __init__(self):
        super().__init__("imu_encoder_odometry")

        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odometry/wheel")
        self.declare_parameter("twist_topic", "erp42/twist/world")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link_2")
        self.declare_parameter("logging", True)

        imu_topic = self.get_parameter("imu_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        twist_topic = self.get_parameter("twist_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.logging = self.get_parameter("logging").value

        # Subscriber 설정
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile=qos_profile_sensor_data
        )
        self.erp_twist_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            twist_topic,
            self.callback_erp_twist,
            qos_profile=qos_profile_system_default,
        )

        # Publisher 설정
        self.odom_pub = self.create_publisher(
            Odometry, odom_topic, qos_profile=qos_profile_system_default
        )

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 상태 변수 초기화
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = euler_from_quaternion(q)
        self.yaw = euler[2]  # Yaw 값 업데이트

    def callback_erp_twist(self, msg):
        current_time = (
            rclpy.time.Time.from_msg(msg.header.stamp).seconds_nanoseconds()[0]
            + rclpy.time.Time.from_msg(msg.header.stamp).seconds_nanoseconds()[1] * 1e-9
        )

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        self.v_x = msg.twist.twist.linear.x
        self.v_y = msg.twist.twist.linear.y

        # Yaw 값을 고려한 변위 계산 (기본 좌표계가 ROS의 오른손 좌표계임을 감안)
        self.x += (self.v_x) * dt
        self.y += (self.v_y) * dt

        if self.logging:
            self.get_logger().info(f"Position & Time: x={self.x}, y={self.y}, dt={dt}")
            self.get_logger().info(
                f"Velocity & Orientation: vx={self.v_x}, vy={self.v_y}, yaw={self.yaw}"
            )

        # Odometry 메시지 생성 및 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Yaw 값을 quaternion으로 변환
        q = quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom_msg)

        # TF 발행
        self.publish_tf(self.x, self.y, q)

    def publish_tf(self, x, y, q):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_frame_id

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuEncoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
