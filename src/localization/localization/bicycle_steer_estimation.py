import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from erp42_msgs.msg import SerialFeedBack
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math as m
import time


class Estimate_bicycle_heading(Node):
    def __init__(self):
        super().__init__("bicycle_heading")

        self.create_subscription(
            SerialFeedBack, "erp42_feedback", self.feedback_erp, qos_profile_sensor_data
        )
        self.sub_imu = self.create_subscription(
            Imu, "imu/rotated", self.callback_imu, qos_profile_sensor_data
        )
        self.pub_imu = self.create_publisher(Imu, "steer_yaw", qos_profile_sensor_data)

        self.__L = 1.5
        self.heading = 0.0
        self.flag = False
        self.prev_v = 0.
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        # 이전 steer 값을 저장 (초기값 0으로 설정)
        self.prev_steer = 0.0
        # 최대 steer 변화율 (rad/s) - 필요에 따라 조정하세요.
        self.max_steer_rate = 0.5

    def callback_imu(self, msg):
        if not self.flag:
            _, _, self.heading = euler_from_quaternion([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])
            self.flag = True

    def limit_steer_rate(self, current_steer, prev_steer, dt, max_steer_rate):
        """
        현재 steer 값을 이전 값과 비교하여 dt 동안 최대 max_steer_rate 만큼만 변화하도록 제한합니다.
        """
        delta_steer = current_steer - prev_steer
        max_delta = max_steer_rate * dt

        if delta_steer > max_delta:
            limited_steer = prev_steer + max_delta
        elif delta_steer < -max_delta:
            limited_steer = prev_steer - max_delta
        else:
            limited_steer = current_steer

        return limited_steer

    def feedback_erp(self, msg):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        dt = current_time - self.last_time
        self.last_time = current_time

        v = self.lpf(msg,0.8)  # m/s
        print(v)
        theoretical_steer = msg.steer  # 이론상의 steer (rad)
        if theoretical_steer <-0.1:
            theoretical_steer -= 0.15

        # 최대 steer 변화율 제한 적용
        limited_steer = self.limit_steer_rate(theoretical_steer, self.prev_steer, dt, self.max_steer_rate)
        self.prev_steer = limited_steer  # 다음 계산을 위해 업데이트
        # 제한된 steer 값을 사용해 heading 업데이트
        self.heading, w_z = self.update_orientation(self.heading, v, limited_steer, dt)

        q = quaternion_from_euler(0, 0, self.heading)
        self.get_logger().info(f"Heading: {self.heading} dt: {dt}")

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = w_z

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        self.pub_imu.publish(imu_msg)

    def update_orientation(self, theta, v, steer, dt):
        """
        단순한 바이시클 모델을 이용해 heading(orientation)을 업데이트합니다.
        theta: 현재 heading (rad)
        v: 전진 속도 (m/s)
        steer: 조향각 (rad) - 여기서는 제한된 steer 값 사용
        dt: 시간 간격 (s)
        """
        w_z = (v / self.__L) * m.tan(steer)
        dtheta = w_z * dt
        theta_new = theta + dtheta
        # -pi ~ pi 범위로 정규화
        theta_new = m.atan2(m.sin(theta_new), m.cos(theta_new))
        return theta_new, w_z
    
    def lpf(self, msg, alpha):
        v = msg.speed
        v_lpf = alpha * self.prev_v + (1 - alpha) * v
        self.prev_v = v_lpf
        return v_lpf


def main(args=None):
    rclpy.init(args=args)
    node = Estimate_bicycle_heading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
