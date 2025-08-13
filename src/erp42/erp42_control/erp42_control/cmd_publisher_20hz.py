#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import ControlMessage, SerialFeedBack
import math
import numpy as np
class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
        # self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 1.0).value
        # self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.05).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )

    def PIDControl(self, speed, desired_value, min = 0, max = 25):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (
            self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt

        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)
        if self.i_err > 5.0:
            self.i_err = 5.0
        if self.i_err < -5.0:
            self.i_err = -5.0

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)

        return int(np.clip(self.speed, min, max))


class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher_20hz')
        
        self.pub = self.create_publisher(ControlMessage, '/cmd_msg', 10)
        self.create_subscription(
            SerialFeedBack,
            "erp42_feedback",
            self.callback_erp,
            qos_profile=qos_profile_system_default,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz → 0.05초 주기
        self.pid = PID(self)

        self.count = 0.0
        self.v = 0.0
        self.get_logger().info("CmdPublisher node started at 20Hz")

    def callback_erp(self, msg):
        self.speed = msg
        self.v = msg.speed  # TODO speed EKF result로 받기
        
    def timer_callback(self):
        msg = ControlMessage()
        
        # 실험용 speed/steer 값
        msg.mora = 1          # A 모드
        msg.estop = 0
        msg.gear = 2          # D 기어
        msg.speed = 50 # kph
        
        # 예: sin 파형으로 steer 변화 (약 ±0.2 rad)
        msg.steer = int(28 * math.sin(3.0 * self.count)) 
        
        msg.brake = 0
        msg.alive = 0
        
        self.pub.publish(msg)
        
        self.count += 0.05  # 시간 증가 (20Hz 주기)
        self.get_logger().info(f"Publishing cmd: speed={msg.speed:.2f}, steer={msg.steer:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping cmd publisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
