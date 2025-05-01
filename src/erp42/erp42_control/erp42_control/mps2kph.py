#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import ControlMessage, SerialFeedBack
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
import math as m
import numpy as np

class mps2kph(Node):
    def __init__(self):
        super().__init__('mps2kph_node')

        self.cmd_pub = self.create_publisher(ControlMessage, "cmd_msg", 1)
        self.cmd_msg = ControlMessage()

        self.msf_sub = self.create_subscription(AckermannDriveStamped, "msfpeed", self.msf_callback, 1)
        self.msf_msg = AckermannDriveStamped()
        
        self.create_subscription(SerialFeedBack, "/localization/kinematic_state", self.odom_callback, 1)   
        
        # PID 제어에 필요한 변수들 초기화
        self.p_gain = 2.07
        self.i_gain = 0.85
        # self.d_gain = 0.0
        
        self.p_err = 0.0
        self.i_err = 0.0 
        self.d_err = 0.0
        
        self.current = self.get_clock().now().seconds_nanoseconds()[0] + (self.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.init_time = self.current
        self.last = self.current
        self.dt = 0.0
        
        self.time = []
        self.odom_msg = Odometry()
        self.odom_v = 0.0


        self.v = 0     


    
    def msf_callback(self, msg : AckermannDriveStamped):
        self.msf_msg.speed = msg.drive.speed
        self.msf_msg.steer = msg.drive.steering_angle

        desired_speed = self.msf_msg.speed * 3.6
        desired_steer = m.degrees(self.msf_msg.steer)
        self.get_logger().info(f"KPH speed : {desired_speed}, DEG steer : {desired_steer}")

        pid_speed = self.PIDControl(desired_speed)
        self.cmd_msg.speed = int(pid_speed * 10 )
        self.get_logger().warn(f"f2i loss : {pid_speed * 10 - self.cmd_msg.speed}")
        self.cmd_msg.steer = int(m.degrees(-1) * desired_steer)
        self.get_logger().info(f"mps speed : {self.cmd_msg.speed}, deg steer : {self.cmd_msg.steer}")
    

    def odom_callback(self, msg : Odometry):
        self.v = self.odom_msg.twist.twist.linear.x
        self.get_logger().info(f"ODOM speed : {self.odom_v}")

    def PIDControl(self, desired_value, min = 0 , max = 25): 
        self.current = self.get_clock().now().seconds_nanoseconds()[0] + (self.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.time.append(self.current - self.init_time)

        self.dt = self.current - self.last
        err = desired_value - self.v
        # delta_err = err - self.prev_err
        # print(err)
        
        self.p_err = err
        self.i_err += self.p_err * self.dt  * (0.0 if self.v == 0 else 1.0)
        # self.d_err = delta_err / self.dt

        self.last = self.current

        speed = self.v + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        # speed = self.v + (self.p_gain * self.p_err) + (self.i_gain * self.i_err) + (self.d_gain * self.d_err)

        self.final_speed = int(np.clip(speed, min, max))
        # print(self.final_speed)

        self.prev_err = err

        return self.final_speed
        
        
def main(args=None):
    rclpy.init(args=args)
    node = mps2kph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()