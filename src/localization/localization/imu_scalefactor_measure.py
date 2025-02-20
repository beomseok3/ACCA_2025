import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Temperature
import math as m
from tf_transformations import euler_from_quaternion

import threading

# TOOD: add threading to make it work efficiently


class ImuBiasMeasure(Node):

    def __init__(self):
        super().__init__("imu_bias_measure")
        self.subscription = self.create_subscription(
            Imu, "imu/data", self.listener_callback, qos_profile=qos_profile_sensor_data
        )

        # self.subscription_temperature = self.create_subscription(
        #     Temperature,
        #     "temperature",
        #     self.temperature_listener_callback,
        #     qos_profile=qos_profile_sensor_data,
        # )

        self.current_time = None
        self.last_time = None
        self.last_anguar_velocity = None
        self.flag = False
        self.total_time = 0.0
        self.total_temperature = 0.0
        self.bias = 0.0

    def listener_callback(self, msg):
        """integrate omega and measure bias

        Note:
            the bias is calculated by integrating omega over time
            data is assumed to be in stop
            discrete integration method is tustin-method
        """

        print(f"imu_data: {msg.angular_velocity.z}")

        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = self.current_time
            print(f"{self.current_time} {self.last_time}")

        if self.last_anguar_velocity is None:
            self.last_anguar_velocity = msg.angular_velocity.z
            return

        self.bias += (
            (0.5)
            * (msg.angular_velocity.z + self.last_anguar_velocity)
            * (self.current_time - self.last_time)
        )

        self.total_time += self.current_time - self.last_time

        self.last_anguar_velocity = msg.angular_velocity.z
        self.last_time = self.current_time

        bias = -self.bias
        scale_factor = 2 * m.pi - bias
        # time = self.total_time
        # avg_bias = self.bias / time
        _, _, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        self.get_logger().info(f"degree of yaw is: {m.degrees(yaw)}")
        self.get_logger().info(
            f"The total dyaw is: {bias}, and the scale factor is: {scale_factor}"
        )
        # self.make_txt(time, avg_bias)

    # def temperature_listener_callback(self, msg):

    #     self.get_logger().info(f"Temperature: {msg.temperature}")

    #     self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    #     if self.last_time is None:
    #         self.last_time = self.current_time
    #         print(f"{self.current_time} {self.last_time}")
    #         return

    #     self.total_temperature += msg.temperature * (self.current_time - self.last_time)
    #     self.total_time += self.current_time - self.last_time

    #     self.last_time = self.current_time

    #     time = self.total_time
    #     avg_temperature = self.total_temperature / time
    #     self.get_logger().info(f"The average temperature is: {avg_temperature}")
    #     self.make_txt(time, avg_temperature)

    def make_txt(self, time, value):

        f = open("/home/ps/imu_bias/imu7_bias_inner_test.txt", "a")
        data = "{},{}\n".format(time, value)
        f.write(data)
        f.close()


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasMeasure()
    rclpy.spin(node)
    # thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # thread.start()
    # rate = node.create_rate(10)

    # while rclpy.ok():
    #     try:
    #         print("!")
    #     except Exception as ex:
    #         print(ex)
    #     rate.sleep()
    node.destroy_node()
    rclpy.shutdown()
