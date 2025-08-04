import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from erp42_msgs.msg import ControlMessage
from std_msgs.msg import Float32

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.get_logger().info("MPC Node has been started.")
        
        # Define QoS profile for the publisher
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.create_subscription(ControlMessage, "cmd_msg",self.callback, qos_profile)
        # Create a publisher with the defined QoS profile
        self.publisher_ = self.create_publisher(Float32, 'mpc_topic', qos_profile)
        
        # Timer to publish messages periodically

    def callback(self, msg):
        sp = msg.speed
        sp = sp / 36

        msg =Float32(data = sp)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()