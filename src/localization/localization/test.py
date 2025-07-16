import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TwistWithCovarianceStamped

import math as m

class Gps(Node):
    def __init__(self):
        super().__init__("gps")

        self.create_subscription(TwistWithCovarianceStamped, "ublox_gps_node/fix_velocity", self.callback_gps_vel, qos_profile_system_default)

    def callback_gps_vel(self,msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = m.sqrt(vx**2+vy**2)
        gps_yaw = m.atan2(vy,vx)
        gps_yaw_deg = m.degrees(gps_yaw)
        # self.get_logger().info(f"mps : {v} yaw : {gps_yaw_deg} ")
        print(f"mps : {v} yaw : {gps_yaw_deg} ")
def main(args = None):
    rclpy.init(args=args)
    node = Gps()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()