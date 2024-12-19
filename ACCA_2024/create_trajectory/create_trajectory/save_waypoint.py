#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped

class SaveWaypoint(Node):
    def __init__(self):
        super().__init__("save_waypoint")
        qos_profile = QoSProfile(depth=10)
        self.f = open("/home/gjs/robot_ws/src/create_trajectory/resource/saved_waypoints.txt",mode="w",encoding="UTF-8")
        self.sub = self.create_subscription(PoseStamped, "/planning/mission_planning/goal",self.callback,qos_profile)
        self.num = 1

    def callback(self,msg):
        wpt_num = "[wpt" + str(self.num)  + "]"

        pos_x = str(msg.pose.position.x)
        pos_y = str(msg.pose.position.y)
        pos_z = str(msg.pose.position.z)

        ori_x  = str(msg.pose.orientation.x)
        ori_y  = str(msg.pose.orientation.y)
        ori_z  = str(msg.pose.orientation.z)
        ori_w  = str(msg.pose.orientation.w)

        line = wpt_num + " " + \
               pos_x + " " + \
               pos_y + " " + \
               pos_z + " " + \
               ori_x + " " + \
               ori_y + " " + \
               ori_z + " " + \
               ori_w + "\n"
        print(line)
        self.f.write(line)
        self.num += 1
def main(args = None):
    rclpy.init(args=args)
    sw = SaveWaypoint()
    rclpy.spin(sw)
    sw.destroy_node()
    rclpy.shutdown()
    sw.f.close()


if __name__ == "__main__":
    main()