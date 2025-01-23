#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_adapi_v1_msgs.msg import OperationModeState
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
from array import array

class DummyPublisher(Node):
    def __init__(self):
        super().__init__("dummy_publisher")
        qos_profile = QoSProfile(depth=10)
        self.subscriber = self.create_subscription(Odometry, "/localization/kinematic_state", self.callback_kinematic, qos_profile)
        self.publisher_objects = self.create_publisher(PredictedObjects, "/perception/object_recognition/objects", qos_profile)
        self.publisher_map = self.create_publisher(OccupancyGrid, "/perception/occupancy_grid_map/map", qos_profile)
        # self.timer = self.create_timer(30, self.timer_callback)
       
        # dummy
        self.pub_objects = PredictedObjects()
        self.pub_objects.header.frame_id = "map"
        self.pub_objects.objects = []

        self.pub_map = OccupancyGrid()
        self.pub_map.header.frame_id = "map"
        # self.pub_map.info.map_load_time = self.get_clock().now().to_msg()
        self.pub_map.info.resolution = 0.5
        self.pub_map.info.width = 300
        self.pub_map.info.height = 300
        map_array = array('b', [1] * 90000)
        self.pub_map.data = map_array

    def callback_kinematic(self,msg):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        pose_z = msg.pose.pose.position.z
       
        self.pub_map.info.origin.position.x = pose_x - 150.0
        self.pub_map.info.origin.position.y = pose_y - 150.0
        self.pub_map.info.origin.position.z = pose_z
        self.pub_map.info.origin.orientation.x = 0.0
        self.pub_map.info.origin.orientation.y = 0.0
        self.pub_map.info.origin.orientation.z = 0.0
        self.pub_map.info.origin.orientation.w = 1.0

        self.publisher_objects.publish(self.pub_objects)
        self.publisher_map.publish(self.pub_map)
   
    # def timer_callback(self):


def main(args=None):
    rclpy.init(args=args)
    dum = DummyPublisher()
    dum.get_logger().info("Dummydata publishing........")
    rclpy.spin(dum)
   
    dum.destroy_node()
    rclpy.shutdown()
   

if __name__ == "__main__":
    main()