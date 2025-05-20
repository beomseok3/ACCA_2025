import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from tf_transformations import *

import numpy as np
import math as m


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

        self.total_differ = 0.0
        self.last_time = 0.0
        self.current_time = 0.0
        self.total_time = 0.0
        self.gyro_bias = 0.0

        self.prev_r, self.prev_p, self.prev_y = None, None, None

    def callback_imu(self, msg):

        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time == 0.0:
            self.last_time = self.current_time
            return
            # print(f"{self.current_time} {self.last_time}")

        self.total_time += self.current_time - self.last_time
        self.gyro_bias += m.degrees(
            msg.angular_velocity.z * (self.current_time - self.last_time)
        )

        r, p, y = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )

        r = np.rad2deg(r)
        p = np.rad2deg(p)
        y = np.rad2deg(y)

        if self.prev_r is not None:
            print(
                f"r: {r - self.prev_r}, p: {p - self.prev_p}, y: {y - self.prev_y}"  # heading error시 error 속도 확인하기
            )
            self.total_differ += y - self.prev_y
        # self.get_logger().warn(f"r: {r}, p: {p}, y: {y}")
        print("=====================================")
        print(f"total_differ: {self.total_differ}")
        print(f"gyro differ: {self.gyro_bias}")
        print(f"total_time: {self.total_time}")
        avg_differ = self.total_differ / self.total_time
        print(f"yaw_bias: {avg_differ}")
        self.prev_r = r
        self.prev_p = p
        self.prev_y = y
        self.last_time = self.current_time

        self.pub_deg_yaw.publish(Float32(data=y))
        self.make_txt(self.total_time, avg_differ)

    def make_txt(self, time, value):

        f = open("/home/ps/imu_data/yaw_bias_0319.txt", "a")
        data = "{},{}\n".format(time, value)
        f.write(data)
        f.close()


def main(args=None):
    rclpy.init(args=args)
    node = Imu_test()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
