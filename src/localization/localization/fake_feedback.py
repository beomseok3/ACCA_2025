import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from erp42_msgs.msg import SerialFeedBack


class Fake_feedback(Node):
    def __init__(self):
        super().__init__("fake_feedback")

        self.pub = self.create_publisher(
            SerialFeedBack, "erp42_feedback", qos_profile_system_default
        )

        self.timer = self.create_timer(1 / 20, self.callback_pub)

    def callback_pub(self):
        msg = SerialFeedBack()
        msg.speed = 0.5
        msg.gear = 2

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Fake_feedback()
    rclpy.spin(node)
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
