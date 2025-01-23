#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile

class create_trajectory(Node):
    def __init__ (self):
        super().__init__("create_trajectory")
        qos_profile = QoSProfile(depth=10)
        self.f = open("trajectory.txt","w")
        self.subscriber = self.create_subscription(Trajectory, "/planning/scenario_planning/lane_driving/trajectory", self.subcribe_callback, qos_profile)
        
    def subcribe_callback(self,msg):
        for i in msg.points:
            data = f"{i.pose.position.x}, {i.pose.position.y}\n"
            self.f.write(data)

def main(args=None):
    
    rclpy.init(args=args)
    sub = create_trajectory()
    rclpy.spin_once(sub)
    sub.get_logger().info("Finish")
    sub.destroy_node()
    rclpy.shutdown()
    sub.f.close()

if __name__ == "__main__":
    main()
