import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_system_default
from erp42_msgs.msg import ControlMessage, SerialFeedBack
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

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
        self.publisher_steer = self.create_publisher(Float32, 'cmd_steer', qos_profile)
        self.publisher_erp = self.create_publisher(Float32, 'erp_kph', qos_profile)
        self.create_subscription(Odometry, "localization/kinematic_state", self.callback_erp,qos_profile)
        # self.create_subscription(
        #     SerialFeedBack,
        #     "erp42_feedback",
        #     self.callback_erp_fb,
        #     qos_profile=qos_profile_system_default,
        # )
        # Timer to publish messages periodically
    
    # def callback_erp_fb(self,msg):
    #     erp_sp = msg.speed
    #     erp_sp = erp_sp * 3.6

    #     msg = Float32(data=erp_sp)
    #     self.publisher_erp.publish(msg)



    def callback_erp(self,msg):
        
        erp_sp = msg.twist.twist.linear.x
        erp_sp = erp_sp * 3.6
        msg = Float32(data=erp_sp)
        self.publisher_erp.publish(msg)

    def callback(self, msg):
        sp = msg.speed
        sp = sp / 10

        steer = msg.steer
        real_steer = steer / 1e3
        msg__1 = Float32(data = real_steer)
        self.publisher_steer.publish(msg__1)


        msg =Float32(data = sp)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()