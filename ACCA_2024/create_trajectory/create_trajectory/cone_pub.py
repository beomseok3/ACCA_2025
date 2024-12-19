import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import math as m
import numpy as np

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3
from autoware_perception_msgs.msg import DetectedObjects

class ConPublisher(Node):
    def __init__(self):
        super().__init__("cone_coord_publisher")
        qos_profile = QoSProfile(depth=10)
        
        self.pcl_sub = self.create_subscription(DetectedObjects, "feature_removed", self.callback_pcl, qos_profile )
        self.coord_pub = self.create_publisher(Float32MultiArray, "coord_cone",qos_profile)

    def callback_pcl(self,msg):
        
        coord = []
        array = Float32MultiArray()
        for i in msg.objects:
            coord.extend([i.kinematics.pose_with_covariance.pose.position.x, i.kinematics.pose_with_covariance.pose.position.y, i.kinematics.pose_with_covariance.pose.position.z])
        array.data = coord
        self.coord_pub.publish(array)


def main(args=None):
        rclpy.init(args=args)
        con = ConPublisher()
        rclpy.spin(con)
        rclpy.destroy_node()
        rclpy.shutdown()

    
if __name__ == "__main__":
    main()


    
        

