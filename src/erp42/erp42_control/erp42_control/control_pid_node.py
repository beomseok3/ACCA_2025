import rclpy
from rclpy.node import Node
from erp42_msgs.msg import ControlMessage, SerialFeedBack
import time
import math
import numpy as np
import math as m

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def compute(self, target, current):
        error = target - current
        current_time = time.time()

        if self.prev_time is None:
            dt = 0.1
        else:
            dt = current_time - self.prev_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.prev_time = current_time
        return output

class SteerPIDNode(Node):
    def __init__(self):
        super().__init__('steer_pid_controller')

        self.pid = PIDController(kp=1.2, ki=0.0, kd=0.3)

        self.subscription = self.create_subscription(
            SerialFeedBack,
            'erp42_feedback',
            self.feedback_callback,
            10
        )

        self.publisher = self.create_publisher(
            ControlMessage,
            'cmd_msg',
            10
        )

        # 예시: 원하는 steer 목표값 (rad)
        # 실제로는 외부 노드에서 받아오거나 경로 추종에서 계산된 값이어야 함
        self.target_steer_rad = 0.0  # 초기값

    def feedback_callback(self, msg: SerialFeedBack):
        current_steer_rad = msg.steer  # 라디안 단위
        target_steer_rad = self.target_steer_rad  # 외부에서 설정되었다고 가정

        pid_output_rad = self.pid.compute(target_steer_rad, current_steer_rad)

        # 라디안 → 도 (degree) 변환, int16 제한
        steer_cmd_deg = int(math.degrees(pid_output_rad))
        steer_cmd_deg = np.clip(steer_cmd_deg, m.radians((-1) * 28), m.radians(28))
        cmd_msg = ControlMessage()
        cmd_msg.steer = steer_cmd_deg  # int16

        self.publisher.publish(cmd_msg)

        self.get_logger().info(
            f"[PID] current(rad): {current_steer_rad:.3f}, "
            f"target(rad): {target_steer_rad:.3f}, "
            f"output(deg): {steer_cmd_deg}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SteerPIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
