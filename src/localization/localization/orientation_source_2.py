import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import math


class MagHeadingNode(Node):
    def __init__(self):
        super().__init__("mag_heading_node")
        self.subscription = self.create_subscription(
            MagneticField, "imu/mag", self.mag_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def mag_callback(self, msg):
        # 간단하게 tilt compensation 없이 x, y 만으로 heading 계산
        heading = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
        if heading < 0:
            heading += 2 * math.pi

        # 라디안을 도 단위로 변환
        heading_deg = heading * 180.0 / math.pi
        self.get_logger().info(f"Calculated Heading: {heading_deg:.2f} degrees")


def main(args=None):
    rclpy.init(args=args)
    node = MagHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
