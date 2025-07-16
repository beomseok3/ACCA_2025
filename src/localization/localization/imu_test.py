import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from tf_transformations import *
import math as m
import numpy as np
from rclpy.qos import QoSProfile, qos_profile_sensor_data, qos_profile_system_default



class Test_IMU(Node):
    def __init__(self):
        super().__init__("test_imu")
        qos_profile = QoSProfile(depth = 1)
        self.create_subscription(Imu, "imu/data", self.callback, qos_profile)

        self.prev_roll, self.prev_pitch, self.prev_yaw = None, None, None

    def callback(self, msg):

        # initialize
        roll, pitch, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        # change unit
        roll, pitch, yaw = np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)
        
        self.get_logger().info(f"yaw : {yaw}")
        self.get_logger().info(f"roll pitch : {roll, pitch}")
        
        if self.prev_yaw is not None:
            dr,dp,dy = roll - self.prev_roll, pitch - self.prev_pitch, yaw - self.prev_yaw
             
            if abs(dy) > 0.1:
                self.get_logger().warn(f"yaw_rate : {dy}")
            
            if abs(dp) > 0.1:
                self.get_logger().warn(f"pitch_rate : {dp}")
            
            if abs(dr) > 0.1:
                self.get_logger().warn(f"roll_rate : {dr}")

        # update
        self.prev_roll, self.prev_pitch, self.prev_yaw = roll, pitch, yaw
        
def main(args = None):
    rclpy.init(args=args)
    node = Test_IMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
