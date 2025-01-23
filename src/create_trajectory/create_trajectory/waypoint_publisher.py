#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")
        qos_profile = QoSProfile(depth=10)
        self.waypoint_pub = self.create_publisher(PoseStamped,"/planning/mission_planning/goal",qos_profile)
        self.lane_id_sub = self.create_subscription(Int32,"lane_id",self.callback_lane_id,qos_profile)
        f = open("/home/gjs/robot_ws/src/create_trajectory/resource/saved_waypoints.txt", "r", encoding="UTF-8")
        self.txt_lines = f.readlines()
        self.waypoint_laneid_set = {180:0, 196:1, 220:2} # change key and value of dictionary if you want to change route planning
        self.past_id = None

    def callback_lane_id(self,msg):
        lane_id = msg.data
        state = self.id_state(lane_id)
        if state:
            try:
                self.search_waypoint(lane_id)
            except Exception as ex:
                print(ex)
        else:
            pass
    
    def search_waypoint(self,lane_id):
        line_num = self.waypoint_laneid_set[lane_id]
        line = self.txt_lines[line_num]
        #line : [wpt.n] pos_x pos_y pos_z ori_x ori_y ori_z ori_w
        line_splited = line.split()
        print(lane_id)

        pose = PoseStamped()
        pose.header.frame_id = "map" #error occured, AttributeError: 'property' object has no attribute 'frame_id'

        pose.pose.position.x = float(line_splited[1])
        pose.pose.position.y = float(line_splited[2])
        pose.pose.position.z = float(line_splited[3])

        pose.pose.orientation.x = float(line_splited[4])
        pose.pose.orientation.y = float(line_splited[5])
        pose.pose.orientation.z = float(line_splited[6])
        pose.pose.orientation.w = float(line_splited[7])

        print(pose)

        self.waypoint_pub.publish(pose)

    def id_state(self, lane_id):
        if self.past_id == None:
            self.past_id = lane_id

        if self.past_id == lane_id:
            state = False
        else:
            state  = True
        
        self.past_id = lane_id
        return state

def main(args = None):
    rclpy.init(args=args)
    wp = Waypoint()
    rclpy.spin(wp)
    wp.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
