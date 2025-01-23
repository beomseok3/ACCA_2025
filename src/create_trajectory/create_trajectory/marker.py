import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from autoware_auto_perception_msgs.msg import DetectedObjects
from geometry_msgs.msg import Pose, Vector3

class MarkerPub(Node):
    def __init__(self):
        super().__init__("test_marker")
        qos_profile = QoSProfile(depth=10)
        
        self.yellow_sub = self.create_subscription(Float32MultiArray, "cone_coord", self.callback_yellow, qos_profile)
        self.marker_pub = self.create_publisher(MarkerArray, "yellow_cone",qos_profile)

    def callback_yellow(self,msg):
          data = MarkerArray()
          markers = []
          id = 0
          for i in range(0,len(msg.data),3):
                    marker = Marker()
                    marker.header.frame_id = "velodyne"

                    marker.id = id
                    id += 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD 
                    marker.pose.position.x = msg.data[i]
                    marker.pose.position.y = msg.data[i+1]
                    marker.pose.position.z = msg.data[i+2]

                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0

                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.r = 255.0
                    marker.color.g = 255.0
                    marker.color.b = 0.0
                    marker.color.a = 255.0

                    marker.lifetime.sec = 1

                    markers.append(marker)
          data.markers = markers
          self.marker_pub.publish(data)

def main(args=None):
    rclpy.init(args=args)
    m = MarkerPub()
    rclpy.spin(m)
    m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
          