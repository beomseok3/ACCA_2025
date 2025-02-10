import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
# from tf_transformations import euler_from_quaternion 
# #TODO 의존성 해결

class ErpTwist(Node):
    def __init__(self):
        super().__init__("erp_twist")
        qos_profile = QoSProfile(depth=10)

        self.sub_erp = self.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile)
        # self.sub_imu = self.create_subscription(Imu, "imu/data", self.callback_imu, qos_profile)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, "erp42/twist", qos_profile)
        # self.yaw = None
        self.header = Header()
        self.prev_v = 0.

    def callback_erp(self, msg):
        # if self.yaw is not None:
        header = self.header
        gear = msg.gear
        if gear == 2:
            v = self.lpf(msg, alpha=0.875)
        else:
            v = (-1) * self.lpf(msg, alpha=0.875)
        self.publish_twist(v, header)

    # def callback_imu(self, msg):
    #     quarternion = msg.orientation
    #     _, _, self.yaw = euler_from_quaternion([quarternion.x, quarternion.y, quarternion.z, quarternion.w])

    def publish_twist(self, v, header):
        data = TwistWithCovarianceStamped()

        data.header = header
        data.header.stamp = self.get_clock().now().to_msg()
        data.header.frame_id = "map"
        data.twist.twist.linear.x = v 
        data.twist.twist.linear.y = 0.
        data.twist.twist.linear.z = 0.

        data.twist.twist.angular.x = 0.
        data.twist.twist.angular.y = 0.
        data.twist.twist.angular.z = 0.

        data.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(data)
    

    def lpf(self, msg, alpha):
        v = msg.speed
        v_lpf = alpha * self.prev_v + (1 - alpha) * v
        self.prev_v = v_lpf
        return v_lpf

    def euler_from_quaternion(quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def quaternion_from_euler(roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return qx, qy, qz, qw
    


def main(args=None):
    rclpy.init(args=args)
    node = ErpTwist()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
