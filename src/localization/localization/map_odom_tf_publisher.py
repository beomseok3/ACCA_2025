import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import math as m
import numpy as np
import time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped
from std_msgs.msg import Header, Float32, Float32MultiArray, Empty
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Transform,
    Vector3,
    Quaternion,
    PoseWithCovarianceStamped,
)
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped as PoseStamped2
from nav_msgs.msg import Odometry


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw


class Queue(object):
    def __init__(self, length=10, init=True):
        self.__array = [init for i in range(length)]

    def inputValue(self, value):
        # assert type(value) == bool
        self.__array.append(value)
        del self.__array[0]

    def count(self, flag):
        return self.__array.count(flag)

    def isTrue(self, threshhold=10):
        return self.count(True) >= threshhold

    def isFalse(self, threshhold=10):
        return self.count(False) >= threshhold


class Map_Odom_TF_Publisher(Node):
    def __init__(self):
        super().__init__("map_odom_tf_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("odom", "/odometry/kf"),
                ("pcl_pose", "/pcl_pose"),
                ("pcl_score", "/pcl_score"),
                ("queue_length", 15),
                ("pose_threshold", 0.2),
                ("score_threshold", 0.3),
                ("reinitialize_waiting", 3.0),
                ("valid_threshold", 15),
                ("invalid_threshold", 15),
                ("tf_distance_threshold", 20.0),
                ("reinitialize_count_threshold", 5),
            ],
        )

        self.odom_topic = self.get_parameter("odom").get_parameter_value().string_value
        self.pcl_topic = (
            self.get_parameter("pcl_pose").get_parameter_value().string_value
        )
        self.pcl_score_topic = (
            self.get_parameter("pcl_score").get_parameter_value().string_value
        )
        self.pose_threshold = (
            self.get_parameter("pose_threshold").get_parameter_value().double_value
        )
        self.score_threshold = (
            self.get_parameter("score_threshold").get_parameter_value().double_value
        )
        self.reinitialize_waiting = (
            self.get_parameter("reinitialize_waiting")
            .get_parameter_value()
            .double_value
        )
        self.valid_threshold = (
            self.get_parameter("valid_threshold").get_parameter_value().integer_value
        )
        self.invalid_threshold = (
            self.get_parameter("invalid_threshold").get_parameter_value().integer_value
        )
        self.tf_distance_threshold = (
            self.get_parameter("tf_distance_threshold")
            .get_parameter_value()
            .double_value
        )
        self.reinitialize_count_threshold = (
            self.get_parameter("reinitialize_count_threshold")
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info("odom: {}".format(self.odom_topic))
        self.get_logger().info("pcl_pose: {}".format(self.pcl_topic))
        self.get_logger().info("pcl_score: {}".format(self.pcl_score_topic))
        self.get_logger().info("pose_threshold: {}".format(self.pose_threshold))
        self.get_logger().info("score_threshold: {}".format(self.score_threshold))
        self.get_logger().info(
            "reinitialize_waiting: {}".format(self.reinitialize_waiting)
        )
        self.get_logger().info("valid_threshold: {}".format(self.valid_threshold))
        self.get_logger().info("invalid_threshold: {}".format(self.invalid_threshold))
        self.get_logger().info(
            "tf_distance_threshold: {}".format(self.tf_distance_threshold)
        )
        self.get_logger().info(
            "reinitialize_count_threshold: {}".format(self.reinitialize_count_threshold)
        )

        self.odom = Odometry()
        self.pcl = PoseWithCovarianceStamped()
        self.predicted_pcl = PoseWithCovarianceStamped()

        self.current_time = time.time()
        self.last_time = time.time()

        self.data = [99.9, 99.9]
        self.score = 99.9

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            callback=self.odom_update_state,
            qos_profile=qos_profile_system_default,
        )
        self.pcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pcl_topic,
            callback=self.pcl_update_state,
            qos_profile=qos_profile_system_default,
        )
        self.score_subscriber = self.create_subscription(
            Float32,
            self.pcl_score_topic,
            callback=self.score_callback,
            qos_profile=qos_profile_system_default,
        )

        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            qos_profile=qos_profile_system_default,
        )

        # TF
        self.buffer = Buffer(node=self, cache_time=Duration(seconds=0.1))
        self.tf_listener = TransformListener(
            self.buffer, self, qos=qos_profile_system_default
        )
        self.tf_publisher = TransformBroadcaster(self, qos=qos_profile_system_default)
        self.tf_msg = TransformStamped()

        self.queue_length = (
            self.get_parameter("queue_length").get_parameter_value().integer_value
        )
        self.queue = Queue(init=True, length=self.queue_length)
        self.reinitialize_count = 0

        self.reinitialize_time = time.time()

    def score_callback(self, msg):
        self.score = msg.data

    def odom_update_state(self, msg):
        self.odom = msg

    def pcl_update_state(self, msg):
        self.predicted_pcl = self.check_validation(pcl_pose=self.pcl)

        self.pcl = msg

        x = abs(self.predicted_pcl.pose.pose.position.x - self.pcl.pose.pose.position.x)
        y = abs(self.predicted_pcl.pose.pose.position.y - self.pcl.pose.pose.position.y)

        self.data = [x, y, self.score]

        validation = (
            self.data[0] < self.pose_threshold
            and self.data[1] < self.pose_threshold
            and self.score < self.score_threshold
        )

        self.queue.inputValue(validation)

        self.get_logger().info(
            "Trust score: {}".format(
                str(round(self.queue.count(True) / self.queue_length, 3))
            )
        )

        if self.queue.isTrue(threshhold=self.valid_threshold):
            self.update_tf()
            self.get_logger().info("Valid TF. Updating...")

        elif self.queue.isFalse(threshhold=self.invalid_threshold):
            reinitialize_dt = time.time() - self.reinitialize_time

            if reinitialize_dt > self.reinitialize_waiting:
                self.reinitialize()
                self.reinitialize_time = time.time()

        self.publish_tf()

    def check_validation(self, pcl_pose):
        self.current_time = time.time()

        dt = self.current_time - self.last_time

        odom_linear_vel = m.sqrt(
            (self.odom.twist.twist.linear.x**2) + (self.odom.twist.twist.linear.y**2)
        )
        odom_angular_vel = self.odom.twist.twist.angular.z

        predicted_pcl_pose = PoseWithCovarianceStamped()
        predicted_pcl_pose.header = Header(frame_id="map", stamp=Time().to_msg())

        _, _, pcl_yaw = euler_from_quaternion(
            [
                pcl_pose.pose.pose.orientation.x,
                pcl_pose.pose.pose.orientation.y,
                pcl_pose.pose.pose.orientation.z,
                pcl_pose.pose.pose.orientation.w,
            ]
        )

        predicted_pcl_yaw = pcl_yaw + (odom_angular_vel * dt)

        predicted_pcl_quat = quaternion_from_euler(0.0, 0.0, predicted_pcl_yaw)

        predicted_pcl_pose.pose.pose.position.x = pcl_pose.pose.pose.position.x + (
            odom_linear_vel * m.cos(predicted_pcl_yaw) * dt
        )
        predicted_pcl_pose.pose.pose.position.y = pcl_pose.pose.pose.position.y + (
            odom_linear_vel * m.sin(predicted_pcl_yaw) * dt
        )

        predicted_pcl_pose.pose.pose.orientation.x = predicted_pcl_quat[0]
        predicted_pcl_pose.pose.pose.orientation.y = predicted_pcl_quat[1]
        predicted_pcl_pose.pose.pose.orientation.z = predicted_pcl_quat[2]
        predicted_pcl_pose.pose.pose.orientation.w = predicted_pcl_quat[3]

        self.last_time = self.current_time

        return predicted_pcl_pose

    # Update tf_msg via odom & pcl
    def update_tf(self):
        p1 = self.odom.pose.pose
        p2 = self.pcl.pose.pose

        new_tf = self.calculate_transform(p1, p2)

        dx = new_tf.translation.x - self.tf_msg.transform.translation.x
        dy = new_tf.translation.y - self.tf_msg.transform.translation.y

        ds = m.sqrt((dx**2) + (dy**2))

        if self.tf_msg.child_frame_id == "odom":
            if ds > self.tf_distance_threshold:
                if self.reinitialize_count <= self.reinitialize_count_threshold:
                    self.get_logger().warn("Detected invalid TF. Ignoring updated TF.")
                    self.queue.inputValue(False)
                    self.reinitialize_count += 1
                    return

                elif self.reinitialize_count > self.reinitialize_count_threshold:
                    self.reinitialize_count = 0

            else:
                # correct tf. reinitalcount reset
                self.reinitialize_count = 0

        self.tf_msg = TransformStamped(
            header=Header(frame_id="map", stamp=Time().to_msg()),
            child_frame_id="odom",
            transform=new_tf,
        )

    # Calculate Transform via odom & pcl
    def calculate_transform(self, p1, p2):
        _, _, yaw1 = euler_from_quaternion(
            [
                p1.orientation.x,
                p1.orientation.y,
                p1.orientation.z,
                p1.orientation.w,
            ]
        )

        _, _, yaw2 = euler_from_quaternion(
            [
                p2.orientation.x,
                p2.orientation.y,
                p2.orientation.z,
                p2.orientation.w,
            ]
        )

        trans = [
            p2.position.x
            - (
                (p1.position.x * m.cos(yaw2 - yaw1))
                - (p1.position.y * m.sin(yaw2 - yaw1))
            ),
            p2.position.y
            - (
                (p1.position.x * m.sin(yaw2 - yaw1))
                + (p1.position.y * m.cos(yaw2 - yaw1))
            ),
        ]

        rot = quaternion_from_euler(0.0, 0.0, yaw2 - yaw1)

        return Transform(
            translation=Vector3(x=trans[0], y=trans[1], z=0.0),
            rotation=Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3]),
        )

    def publish_tf(self):
        self.tf_msg.header = Header(frame_id="map", stamp=Time().to_msg())
        self.tf_publisher.sendTransform(self.tf_msg)

    def reinitialize(self):
        if self.buffer.can_transform(
            "map",
            "odom",
            Time(),
            Duration(seconds=0.1),
        ):
            ps_odom = PoseStamped2(header=self.odom.header, pose=self.odom.pose.pose)

            ps_odom_on_map = self.buffer.transform(
                ps_odom, "map", Duration(seconds=0.1)
            )

            pose_stamped_cov_map = PoseWithCovarianceStamped()
            pose_stamped_cov_map.header = ps_odom_on_map.header
            pose_stamped_cov_map.pose.pose = ps_odom_on_map.pose

            self.get_logger().info("Reinitializing...")
            self.initpose_pub.publish(pose_stamped_cov_map)

        else:
            self.get_logger().warn("Cannot lookup transform")


def main():
    rclpy.init(args=None)

    node = Map_Odom_TF_Publisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
