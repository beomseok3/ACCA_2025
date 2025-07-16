import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
import rclpy.logging
import rclpy.time
import tf2_ros
from abc import *
import numpy as np
import math as m
import pyproj
import time
import threading
# Msgs
from std_msgs.msg import Empty, Header, Float64MultiArray
from sensor_msgs.msg import Imu, NavSatFix
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class Normalizaor(object):
    def __init__(self):
        self.last_value = 0.0
        self.value = 0.0

    def filter(self, value):
        self.value = value

        if abs(self.value - self.last_value) > m.pi:
            if self.last_value < 0.0:
                self.value -= 2
                    

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


def inv(matrix):
    return np.linalg.inv(matrix)


class Kalman(Node):
    def __init__(self):
        super().__init__("kalman_localization_node")

        # Declare Params
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topic", "/odometry/kalman"),
                ("is_publish_tf", True),
                ("frame_id", "odom"),
                ("child_frame_id", "base_link"),
                (
                    "noise",
                    [
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.0,  # x
                        0.0,
                        0.01,
                        0.0,
                        0.0,
                        0.0,  # y
                        0.0,
                        0.0,
                        0.01,
                        0.0,
                        0.0,  # yaw
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                        0.0,  # v
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.01,  # vyaw
                    ],
                ),
                ("gps_topic", "/ublox_gps_node/fix"),
                ("gps_use_covariance", False),
                (
                    "gps_covariance",
                    [
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        99.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ],
                ),
                ("encoder_topic", "/erp42_feedback"),
                (
                    "encoder_covariance",
                    [
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        99.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        99.0,
                    ],
                ),
                ("imu_topic", "/imu/data"),
                ("imu_use_covariance", False),
                (
                    "imu_covariance",
                    [
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        99.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ],
                ),
                ("logging", False),
            ],
        )

        # Declare Instances : GPS, ERP42, XSENS, KALMAN
        self.gps = Ublox(
            self,
            NavSatFix,
            self.get_parameter("gps_topic").get_parameter_value().string_value,
        )
        self.erp42 = ERP42(
            self,
            SerialFeedBack,
            self.get_parameter("encoder_topic").get_parameter_value().string_value,
        )
        self.xsens = Xsens(
            self,
            Imu,
            self.get_parameter("imu_topic").get_parameter_value().string_value,
        )

        # Topic Settings
        self.is_publish_tf = (
            self.get_parameter("is_publish_tf").get_parameter_value().bool_value
        )
        self.tf_publisher = tf2_ros.TransformBroadcaster(
            self, qos=qos_profile_system_default
        )
        self.odom_publisher = self.create_publisher(
            topic=self.get_parameter("topic").get_parameter_value().string_value,
            msg_type=Odometry,
            qos_profile=qos_profile_system_default,
        )

        # Variables
        self.logging = self.get_parameter("logging").get_parameter_value().bool_value
        self.last_time = time.time()
        self.dt = 0.0

        # State Matrix
        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
                0.0,  # vyaw
            ]
        )

        # Model
        self.A = np.array(
            [
                [1, 0, 0, m.cos(self.x[2]) * self.dt, 0],  # x
                [0, 1, 0, m.sin(self.x[2]) * self.dt, 0],  # y
                [0, 0, 1, 0, self.dt],  # yaw
                [0, 0, 0, 1, 0],  # v
                [0, 0, 0, 0, 1],  # vyaw
            ]
        )

        # Conv Matrix
        self.P = np.array(
            [
                [0.01, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.01, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.01, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.01, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.01],
            ]
        )

        # Noise Matrix
        self.Q = np.reshape(
            a=np.array(
                self.get_parameter("noise").get_parameter_value().double_array_value
            ),
            newshape=(5, 5),
        )

        # Loop
        self.create_timer(float(1 / 30.0), callback=self.main)

    def filter(self):
        current_time = time.time()

        self.A = np.array(
            [
                [1, 0, 0, m.cos(self.x[2]) * self.dt, 0],  # x
                [0, 1, 0, m.sin(self.x[2]) * self.dt, 0],  # y
                [0, 0, 1, 0, self.dt],  # yaw
                [0, 0, 0, 1, 0],  # v
                [0, 0, 0, 0, 1],  # vyaw
            ]
        )

        u_k = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
                0.0,  # vyaw
            ]
        )  # Control Vector : TO DO

        R_t = np.array(
            [
                [0.001, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.001, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )

        R = R_t + self.gps.cov + self.xsens.cov + self.erp42.cov  # Convariances

        x_k = np.dot(self.A, self.x) + u_k  # Predicted State

        P_k = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q  # Predicted Convariance

        # Kalman Gain
        S_gps = inv(np.dot(np.dot(self.gps.H, P_k), self.gps.H.T) + R) #()
        K_gps = np.dot(
            np.dot(P_k, self.gps.H.T),
            S_gps,
        )

        S_xsens = inv(np.dot(np.dot(self.xsens.H, P_k), self.xsens.H.T) + R)
        K_xsens = np.dot(
            np.dot(P_k, self.xsens.H.T),
            S_xsens,
        )

        S_erp42 = inv(np.dot(np.dot(self.erp42.H, P_k), self.erp42.H.T) + R)
        K_erp42 = np.dot(
            np.dot(P_k, self.erp42.H.T),
            S_erp42,
        )

        K = (
            np.dot(K_gps, inv(S_gps))
            + np.dot(K_xsens, inv(S_xsens))
            + np.dot(K_erp42, inv(S_erp42))
        )

        # Updated X
        X = (
            x_k
            + np.dot(K_gps, (self.gps.x - np.dot(self.gps.H, x_k)))
            + np.dot(K_xsens, (self.xsens.x - np.dot(self.xsens.H, x_k)))
            + np.dot(K_erp42, (self.erp42.x - np.dot(self.erp42.H, x_k)))
        )
        self.x = X

        H = np.array(
            [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )

        # Update Covariance
        self.P = P_k - np.dot(np.dot(K, H), P_k)
        self.P[0][0] = 0.1
        self.P[1][1] = 0.1

        self.dt = current_time - self.last_time
        self.last_time = current_time

        return self.x, self.P

    def getX(self):
        msg = "\nX: {}\tY: {}\tYAW: {}\tV:{}\tVYAW: {}".format(
            round(self.x[0], 3),
            round(self.x[1], 3),
            round(self.x[2], 3),
            round(self.x[3], 3),
            round(self.x[4], 3),
        )

        return msg

    def getOdometry(self):
        msg = Odometry()

        msg.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]

        x, y, z, w = quaternion_from_euler(0.0, 0.0, self.x[2])
        msg.pose.pose.orientation.x = x
        msg.pose.pose.orientation.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w

        msg.pose.covariance[0] = self.P[0][0]
        msg.pose.covariance[7] = self.P[1][1]

        msg.twist.twist.linear.x = self.x[3] * m.cos(self.x[2])
        msg.twist.twist.linear.y = self.x[3] * m.sin(self.x[2])

        msg.twist.twist.angular.z = self.x[4]

        return msg

    def getTF(self):
        tf_msg = tf2_ros.TransformStamped()

        tf_msg.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        tf_msg.transform.translation.x = self.x[0]
        tf_msg.transform.translation.y = self.x[1]
        tf_msg.transform.translation.z = 0.0

        x, y, z, w = quaternion_from_euler(0.0, 0.0, self.x[2])
        tf_msg.transform.rotation.x = x
        tf_msg.transform.rotation.y = y
        tf_msg.transform.rotation.z = z
        tf_msg.transform.rotation.w = w

        return tf_msg

    def main(self):
        x, P = self.filter()
        odometry = self.getOdometry()
        self.odom_publisher.publish(odometry)

        if self.logging is True:
            self.get_logger().info(self.getX())
            self.get_logger().info(
                "\n{}\t{}".format(round(self.gps.x[3], 3), round(self.erp42.x[3], 3))
            )

        if self.is_publish_tf is True:
            tf_msg = self.getTF()
            self.tf_publisher.sendTransform(tf_msg)


class Sensor(object):
    def __init__(self, node, dtype, topic):
        self.node = node

        self.use_covariance = False

        # Subscriber
        self.subscriber = self.node.create_subscription(
            dtype, topic, self.callback, qos_profile_sensor_data
        )

        # Sensing Data
        self.x = np.array(
            [
                0.0,  # x
                0.0,  # y
                0.0,  # yaw
                0.0,  # v
                0.0,  # vyaw
            ]
        )

        # Conv Matrix
        self.cov = np.array(
            [
                [0.1, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.1, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.1, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.1, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.1],
            ]
        )

        # Transform Matrix : Prediction - Sensor
        self.H = np.array(
            [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )

    @abstractmethod
    def callback(self, msg):
        pass


# UBLOX
class Ublox(Sensor):
    def __init__(self, node, dtype, topic):
        super().__init__(node, dtype, topic)

        self.use_covariance = (
            self.node.get_parameter("gps_use_covariance")
            .get_parameter_value()
            .bool_value
        )

        self.H = np.array(
            [
                [0, 0, 0, 0, 0],  # x
                [0, 0, 0, 0, 0],  # y
                [0, 0, 0, 0, 0],  # yaw
                [0, 0, 0, 1, 0],  # v
                [0, 0, 0, 0, 0],  # vyaw
            ]
        )

        self.gps = pyproj.CRS("epsg:4326")  # lat, log
        self.tm = pyproj.CRS("epsg:2097")  # m
        self.transformer = pyproj.Transformer.from_crs(self.gps, self.tm)

        self.last_stamp = None
        self.last_position = None
        self.dt = 0.0


    def calculateDistance(self, p1, p2):
        return m.sqrt(((p1.x - p2.x) ** 2) + ((p1.y - p2.y) ** 2))

    def callback(self, msg):
        # Callback Function: transform latitude & longitude to v

        current_stamp = msg.header.stamp

        # msg = NavSatFix()
        lat = msg.latitude
        lon = msg.longitude

        x, y = self.transformer.transform(lat, lon)
        p = Point()
        p.x = x
        p.y = y
        current_point = p

        if self.last_position is None or self.last_stamp is None:
            self.last_stamp = current_stamp
            self.last_position = current_point
            return

        # else:
        #     self.dt = (rclpy.time.Time.from_msg(current_stamp) - rclpy.time.Time.from_msg(self.last_stamp)).nanoseconds * 10e-10
        #     print(self.dt)
        self.dt = 0.125

        distance = self.calculateDistance(self.last_position, current_point)
        distance = distance if distance > 0.012 else 0.0

        velocity = distance / self.dt if self.dt > 0.0 else self.x[3]

        # self.node.get_logger().info("{}, {}, {}".format(round(velocity, 3), round(distance, 3), round(self.dt, 5)))

        self.x[3] = velocity if (velocity >= 0.0 and velocity <= 10.0) else 0.0

        if self.use_covariance is True:
            self.cov = np.reshape(
                a=np.array(
                    self.node.get_parameter("gps_covariance")
                    .get_parameter_value()
                    .double_array_value
                ),
                newshape=(5, 5),
            )
        else:
            self.cov = np.array(
                [
                    [0.0, 0.0, 0.0, 0.0, 0.0],  # x
                    [0.0, 0.0, 0.0, 0.0, 0.0],  # y
                    [0.0, 0.0, 0.0, 0.0, 0.0],  # yaw
                    [
                        0.0,
                        0.0,
                        0.0,
                        (
                            msg.position_covariance[0] * m.sqrt(111319.490793)
                            if (velocity >= 0.0 and velocity <= 10.0)
                            else 99.9
                        ),
                        0.0,
                    ],  # v
                    [0.0, 0.0, 0.0, 0.0, 0.0],  # vyaw
                ]
            )

        self.last_stamp = current_stamp
        self.last_position = current_point


# ERP42 Serial
class ERP42(Sensor):
    def __init__(self, node, dtype, topic):
        super().__init__(node, dtype, topic)

        self.H = np.array(
            [
                [0, 0, 0, 0, 0],  # x
                [0, 0, 0, 0, 0],  # y
                [0, 0, 0, 0, 0],  # yaw
                [0, 0, 0, 1, 0],  # v
                [0, 0, 0, 0, 1],  # vyaw
            ]
        )

        self.last_time = time.time()
        self.dt = 0.0

    def callback(self, msg):
        # Callback Function: velocity & dyaw

        # msg = SerialFeedBack()
        current_time = time.time()
        self.dt = current_time - self.last_time

        self.x[3] = msg.speed
        self.x[4] = (msg.speed / 1.040) * m.tan(msg.steer)
        self.cov = np.reshape(
            a=np.array(
                self.node.get_parameter("encoder_covariance")
                .get_parameter_value()
                .double_array_value
            ),
            newshape=(5, 5),
        )

        self.last_time = current_time


class Xsens(Sensor):
    def __init__(self, node, dtype, topic):
        super().__init__(node, dtype, topic)

        self.use_covariance = (
            self.node.get_parameter("imu_use_covariance")
            .get_parameter_value()
            .bool_value
        )

        self.H = np.array(
            [
                [0, 0, 0, 0, 0],  # x
                [0, 0, 0, 0, 0],  # y
                [0, 0, 1, 0, 0],  # yaw
                [0, 0, 0, 0, 0],  # v
                [0, 0, 0, 0, 0],  # vyaw
            ]
        )

        self.initial_yaw = None

        self.angle_normalizor = Normalizaor()

    def callback(self, msg):
        # Callback Function: yaw

        # msg = Imu()
        _, _, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )

        if self.initial_yaw is None:
            self.initial_yaw = yaw

        # yaw = self.angle_normalizor.filter(yaw - self.initial_yaw)
        # self.node.get_logger().info(str(yaw))
        yaw = yaw - self.initial_yaw

        self.x[2] = yaw

        if self.use_covariance is True:
            self.cov = np.reshape(
                a=np.array(
                    self.node.get_parameter("imu_covariance")
                    .get_parameter_value()
                    .double_array_value
                ),
                newshape=(5, 5),
            )

        self.cov = np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0],  # x
                [0.0, 0.0, 0.0, 0.0, 0.0],  # y
                [0.0, 0.0, msg.orientation_covariance[0], 0.0, 0.0],  # yaw
                [0.0, 0.0, 0.0, 0.0, 0.0],  # v
                [0.0, 0.0, 0.0, 0.0, 0.0],  # vyaw
            ]
        )


def main():
    rclpy.init(args=None)

    node = Kalman()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
