import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import *
import math as m
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class Dfilter(Node):
    def __init__(self):
        super().__init__("dfilter_yaw")
        self.create_subscription(
            Imu, "imu/rotated", self.callback_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "odometry/gps", self.callback_gps, qos_profile_sensor_data
        )

        self.pub_filtered_yaw = self.create_publisher(
            Imu, "imu/filtered_yaw", qos_profile_sensor_data
        )
        self.pub_timer = self.create_timer(1 / 20, self.pfilter)

        self.prev_value = 0.0
        self.value = 0.0
        self.count = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.list_xy = [None] * 5
        self.cnt = 0.0

    def dfilter(self, value):
        dvalue = value - self.prev_value

        if self.value == 0.0:
            self.value = value
            self.prev_value = self.value
            return self.value

        if 0.1 > abs(dvalue) > 0.0004:  # TODO 2022년도에는 작을때가 조건이었음
            self.value -= dvalue
            # self.get_logger().info(f"Filtered Value: {dvalue}")
        else:
            # self.get_logger().info(f"no: {dvalue}")
            pass
        self.value += dvalue

        self.prev_value = value

        return self.value

    def pfilter(self):
        if not any(self.list_xy):
            return

        li = self.list_xy
        if self.check(li):  # dynamic state
            self.cnt = 0
            self.publish(li)
        else:
            # static state
            if li[-1][-1] - li[0][-1] < 0.0003 and self.cnt == 0:
                self.publish(li)
            else:
                if self.cnt == 0:
                    self.dyaw = li[-1][-1] - li[0][-1]
                    self.cnt += 1
                    li[-1][-1] = li[-1][-1] - self.dyaw
                    self.publish(li)
                else:
                    self.dyaw += li[-1][-1] - li[0][-1]
                    self.cnt += 1
                    li[-1][-1] = li[-1][-1] - self.dyaw
                    self.publish(li)

    def publish(self, list):
        msg = list[-1][3]
        quaternion = quaternion_from_euler([0, 0, list[-1][-1]])
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        self.pub_filtered_yaw.publish(msg)

    def callback_imu(self, msg):
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.list_xy.append((self.current_x, self.current_y, msg, yaw))
        del self.list_xy[0]
        print("================================")
        print(f"{self.list_xy}")
        print("================================")

    def callback_gps(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def check(self, a):

        a_np = np.array(a)
        x = a_np[:, 0]
        print(x)
        y = a_np[:, 1]
        print(y)
        dx_ = np.diff(x)
        print(dx_)
        dy_ = np.diff(y)
        print(dy_)
        ds = np.sqrt(dx_**2 + dy_**2)
        print(ds)
        s = np.concatenate([[0], np.cumsum(ds)])
        print(s[-1])
        if s[-1] > 0.1:
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = Dfilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
