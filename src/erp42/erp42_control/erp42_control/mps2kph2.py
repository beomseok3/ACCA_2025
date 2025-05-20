#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import ControlMessage, SerialFeedBack, AckermannDriveStamped
# from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
import math as m
import numpy as np




class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    
    def PIDControl(self, speed, desired_value, min, max):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, min, max))
    

class mps2kph(Node):
    def __init__(self):
        super().__init__('mps2kph_node')

        self.cmd_pub = self.create_publisher(ControlMessage, "cmd_msg", 1)

        self.create_subscription(AckermannDriveStamped, "mpc_cmd", self.msf_callback, 1)
        
        self.create_subscription(Odometry, "/localization/kinematic_state", self.odom_callback, 1)   
        
        self.kph_msg = ControlMessage()
        self.mps_msg = AckermannDriveStamped()
        self.odometry_v = 0

        self.pid = PID(self)
    
    def msf_callback(self, msg : AckermannDriveStamped):
        self.mps_msg.speed = msg.speed
        self.mps_msg.steering_angle = msg.steering_angle

        # print(self.mps_msg.drive.speed)

        desired_speed_kph = self.mps_msg.speed * 3.6
        desired_steer_deg = m.degrees((-1)*self.mps_msg.steering_angle)
        self.get_logger().info(f"KPH speed : {desired_speed_kph}, ")

        pid_speed = self.pid.PIDControl(self.odometry_v * 3.6, desired_speed_kph, 0, 25)
        self.get_logger().info(f"PID speed : {pid_speed}")
        
        brake = self.cacluate_brake(self.odometry_v * 3.6, desired_speed_kph) # brake 조정

        # self.cmd_msg = ControlMessage()
        self.kph_msg.speed = int(pid_speed * 10 )
        self.kph_msg.steer = int(desired_steer_deg)
        self.kph_msg.gear = 2
        self.kph_msg.brake = int(brake) 
        self.cmd_pub.publish(self.kph_msg)

    def odom_callback(self, msg : Odometry):
        self.odometry_v = msg.twist.twist.linear.x
        # self.get_logger().info(f"ODOM speed mps : {self.odometry_v * 3.6}")
    
    def cacluate_brake(self, v, adapted_speed): # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if v  >= adapted_speed:
            brake = (abs(v - adapted_speed) / 20.0) * 200
            brake = np.clip(brake, 0, 100)
        else:
            brake = 0
        return brake
def main(args=None):
    rclpy.init(args=args)
    node = mps2kph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()