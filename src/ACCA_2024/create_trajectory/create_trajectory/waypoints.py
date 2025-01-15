#!/usr/bin/env python3

import rclpy
import statistics
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import PathWithLaneId
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile


class waypoint(Node):
    def __init__(self):
        super().__init__("current_id")
        qos_profile = QoSProfile(depth=10)
        self.subscriber = self.create_subscription(PathWithLaneId,"/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",self.lane_callback,qos_profile)
        self.publisher = self.create_publisher(Int32, "lane_id",qos_profile)

    def lane_callback(self,msg):
        lanes = []
        c = 0
        # print(msg.points[0:11])
        for i in msg.points[0:11]: #### 왜 5번밖에 반복 안하지??????????/
            # print(i.lane_ids)
            lane_number = len(i.lane_ids)
            if lane_number == 1:
                lanes.append(i.lane_ids[0]) #각 점의 lane_id
            elif lane_number == 0:
                c +=1
                print("empty_lane: ",c)
            else:
                lanes.append(i.lane_ids[-1])
        
        current_id = statistics.mode(lanes)    
        pub_id = Int32()
        pub_id.data = current_id
        print("lane_id :", current_id)
        self.publisher.publish(pub_id)
                


def main(args=None):
    rclpy.init(args=args)
    sub = waypoint()
    rclpy.spin(sub)
    sub.get_logger().info("Finish")
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__name__":
    main()