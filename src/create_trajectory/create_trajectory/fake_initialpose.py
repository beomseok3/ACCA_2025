#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class InitialposePublisher(Node):
    def __init__(self):
        super.__init__("fake_initial")
        qos_profile = QoSProfile(depth = 10)

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.callback_initialpose, qos_profile)
        self.local_pub = self.create_publisher(Odometry, '/localization/kinematic_state', qos_profile)

    def callback_initialpose(self,msg):
        data = Odometry()

        data.header.stamp = self.get_clock().now().to_msg()
        data.header.frame_id = 'map'
        data.child_frame_id = 'base_link'
        
        data.pose.pose.position = msg.pose.pose.position
        data.pose.pose.orientation = msg.pose.pose.orientation

        data.twist.twist.linear.x = 0.0
        data.twist.twist.linear.y = 0.0
        data.twist.twist.linear.z = 0.0

        data.twist.twist.angular.x = 0.0
        data.twist.twist.angular.y = 0.0
        data.twist.twist.angular.z = 0.0

        data.twist.covariance = [0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0]
        
        self.local_pub.publish(data)

def main(args=None):
    rclpy.init(args=args)
    ip = InitialposePublisher()

    try:
        rclpy.spin(ip)
    except KeyboardInterrupt:
        ip.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        ip.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()