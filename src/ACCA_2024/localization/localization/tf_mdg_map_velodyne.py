import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped


class VelodyneTF(object):
    def __init__(self, node):
        self.node = node

        self.subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/pcl_pose",
            callback=self.callback,
            qos_profile=10,
        )
        self.tf_publisher = tf2_ros.TransformBroadcaster(self.node, qos=10)

    def callback(self, msg):
        # msg = PoseWithCovarianceStamped()

        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.frame_id = "map"
        tf_msg.header.stamp = rclpy.time.Time().to_msg()

        tf_msg.child_frame_id = "velodyne"
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z

        tf_msg.transform.rotation.x = msg.pose.pose.orientation.x
        tf_msg.transform.rotation.y = msg.pose.pose.orientation.y
        tf_msg.transform.rotation.z = msg.pose.pose.orientation.z
        tf_msg.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_publisher.sendTransform(tf_msg)


def main():
    rclpy.init(args=None)

    node = rclpy.create_node("velodyne_tf_node")

    velodyne_tf = VelodyneTF(node)

    try:
        rclpy.spin(node)

    except Exception as ex:
        print(ex)
    finally:
        node.destroy_node()
        rclpy.shutdown()

main()