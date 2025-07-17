from erp42_msgs.msg import ControlMessage, SerialFeedBack  # SerialFeedBack 추가
from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_system_default
import math as m


class Erp(Node):
    def __init__(self):
        super().__init__("erp")

        self.pub = self.create_publisher(
            ControlMessage, "cmd_msg", qos_profile_system_default
        )

        self.timer = self.create_timer(1.0, self.callback_erp)

    def callback_erp(self):
        msg = ControlMessage(
            speed=int(3.0) * 10,
            steer=0,
            gear=2,
        )
        print(msg)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Erp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
