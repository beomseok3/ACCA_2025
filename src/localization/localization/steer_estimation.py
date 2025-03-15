import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math as m
import time


class Estimate_bicycle_heaing(Node):
    def __init__(self):
        super().__init__("bicycle_heading")

        self.create_subscription(
            SerialFeedBack, "erp42_feedback", self.feedback_erp, qos_profile_sensor_data
        )

        self.pub_heding = self.create_publisher(
            Quaternion, "steer_yaw", qos_profile_sensor_data
        )

        self.__L = 1.24
        self.heading = 0.0
        self.prev_alive = -1
        self.last_time = time.time()

    def feedback_erp(self, msg):
        current_time = time.time()
        # if self.prev_alive == -1:
        #     dt = 0.05  # 초기값 설정
        # else:
        #     alive_diff = msg.alive - self.prev_alive
        #     if alive_diff < 0:
        #         alive_diff += 256  # 255에서 0으로 롤오버 처리
        #     dt = alive_diff * 0.05

        self.prev_alive = msg.alive
        # self.last_time = current_time
        # dt = 0.05
        v = msg.speed  # m/s
        steer = msg.steer  # rad

        self.heading = self.update_orientation(self.heading, v, steer, dt)
        q = quaternion_from_euler(0, 0, self.heading)

        msg = Quaternion()
        msg.x = q[0]
        msg.y = q[1]
        msg.z = q[2]
        msg.w = q[3]
        self.pub_heding.publish(msg)

    def update_orientation(self, theta, v, steer, dt):
        """
        현재 orientation(theta)와 주어진 속도(v), 조향각(steer), 휠베이스(L), 시간 간격(dt)를 이용해 새로운 orientation을 계산합니다.
        theta: 현재 orientation (rad))
        v: 전진 속도 (m/s)
        steer: 조향각 (rad)
        L: 휠베이스 (m)
        dt: 시간 간격 (s)
        """
        dtheta = (v / self.__L) * m.tan(steer) * dt
        theta_new = theta + dtheta

        # orientation을 -pi ~ pi 범위로 정규화 (선택 사항)
        theta_new = m.atan2(m.sin(theta_new), m.cos(theta_new))
        return theta_new


# # 예시 사용
# current_theta = m.radians(10)  # 초기 10도
# v = 5.0  # 5 m/s
# steer = m.radians(5)  # 5도 조향각
# L = 1.04  # 휠베이스 1.04m
# dt = 0.1  # 0.1초 시간 간격


# new_theta = update_orientation(current_theta, v, steer, L, dt)
# print("New orientation (degrees):", m.degrees(new_theta))
def main(args=None):
    rclpy.init(args=args)
    node = Estimate_bicycle_heaing()
    rclpy.spin(node)
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
