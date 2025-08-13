#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from erp42_msgs.msg import ControlMessage, SerialFeedBack
import csv
import time
import os

class SteerLogger(Node):
    def __init__(self):
        super().__init__('steer_logger')

        # 로그 저장할 버퍼
        self.log_data = []  # [timestamp, target_steer, actual_steer]

        # ERP42 ROS 토픽 구독
        self.sub_cmd = self.create_subscription(
            ControlMessage, '/cmd_msg', self.cmd_callback, 10
        )
        self.sub_fb = self.create_subscription(
            SerialFeedBack, '/erp42_feedback', self.fb_callback, 10
        )

        # 최신 값 저장
        self.target_steer = None
        self.actual_steer = None

        # 로그 저장 주기 (0.01s = 100Hz)
        self.timer = self.create_timer(0.05, self.log_values)

        # 시작 시간 기록
        self.start_time = time.time()

        self.get_logger().info("SteerLogger node started. Logging target/actual steer...")

    def cmd_callback(self, msg: ControlMessage):
        self.target_steer = msg.steer  # rad 단위라 가정

    def fb_callback(self, msg: SerialFeedBack):
        self.actual_steer = msg.steer  # rad 단위라 가정

    def log_values(self):
        if self.target_steer is not None and self.actual_steer is not None:
            t = time.time() - self.start_time
            self.log_data.append([t, self.target_steer, self.actual_steer])

    def destroy_node(self):
        # CSV 저장
        save_path = os.path.join(os.getcwd(), f"steer_log_{int(time.time())}.csv")
        with open(save_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "target_steer_rad", "actual_steer_rad"])
            writer.writerows(self.log_data)
        self.get_logger().info(f"Log saved to {save_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteerLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping logger...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
