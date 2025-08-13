import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import SerialFeedBack, ControlMessage
from std_msgs.msg import Float64

import math as m


class Steer_viewer(Node):
    def __init__(self):
        super().__init__("steer_viewer")

        self.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp_feedback, qos_profile_system_default)
        
        self.pub_steer_deg = self.create_publisher(Float64, "feedback/steer", qos_profile_system_default)

    def callback_erp_feedback(self, msg):
        steer_rad = msg.steer
        steer_deg = m.degrees( (-1) * steer_rad)

        self.pub_steer_deg.publish(Float64(data = steer_deg))

def main(args = None):
    rclpy.init(args = args)
    node = Steer_viewer()
    rclpy.spin(node)

    rclpy.shutdown()
    node.destroy_node()

if __name__ == "__main__":
    main()