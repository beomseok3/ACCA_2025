import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_system_default
from erp42_msgs.msg import ControlMessage, SerialFeedBack
from std_msgs.msg import Float32

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.get_logger().info("MPC Node has been started.")
        
        # Define QoS profile for the publisher
        # qos_profile = QoSProfile(
        #     depth=10,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        # )
        qos_profile = QoSProfile(depth=10)

        
        self.create_subscription(ControlMessage, "cmd_msg",self.callback, qos_profile)
        # Create a publisher with the defined QoS profile
        self.publisher_ = self.create_publisher(Float32, 'cmd_kph', qos_profile)
        self.publisher_erp = self.create_publisher(Float32, 'erp_kph', qos_profile)
        self.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp,qos_profile)
        
        # Timer to publish messages periodically
    def callback_erp(self,msg):
        erp_sp = msg.speed
        erp_sp = erp_sp * 3.6
        msg = Float32(data=erp_sp)
        self.publisher_erp.publish(msg)
    def callback(self, msg):
        sp = msg.speed
        sp = sp / 10

        msg =Float32(data = sp)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()