import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import SerialFeedBack, ControlMessage
import threading


class Check_frequency:
    def __init__(self, node):

        self.pub = node.create_publisher(
            ControlMessage, "cmd_msg", qos_profile_system_default
        )

        self.estop = 0

    def pub_cmd(self):
        msg = ControlMessage()
        msg.gear = 2
        msg.speed = 50
        msg.steer = 28

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("driving")
    f = Check_frequency(node)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20)

    while rclpy.ok():
        try:
            f.pub_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()


if __name__ == "__main__":
    main()