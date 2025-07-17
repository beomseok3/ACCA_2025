#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import matplotlib.pyplot as plt

import sqlite3
import sys
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler


class DB_READER(Node):
    def __init__(self,input_file,table):
        super().__init__("db_reader")
        qos_profile = QoSProfile(depth=10)
        self.pub_path = self.create_publisher(Path, "db_path_3", qos_profile)
        self.db_read(input_file,table)

    def db_read(self,file_path,table):
        
        db_file = file_path
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        cursor.execute(f"""SELECT x,y,yaw FROM {table} """)
        rows = cursor.fetchall()
        print(rows)
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y, yaw in rows:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)
            # else:
            #     continue
            
        self.pub_path.publish(path)
        conn.close()





def main(args=sys.argv):
    rclpy.init(args=args)
    input_file = args[1]
    table = args[2]
    node = DB_READER(input_file,table)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
