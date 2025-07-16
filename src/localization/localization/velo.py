#!/usr/bin/env python3
# velodyne_to_map.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

# import tf2_sensor_msgs



class VelodyneToMap(Node):
    def __init__(self):
        super().__init__('velodyne_to_map')

        # TF buffer & listener -------------------------------------------------
        self.tf_buffer = Buffer(cache_time = rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # I/O topics -----------------------------------------------------------
        self.sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',          # 입력 토픽 이름 (필요하면 수정)
            self.pointcloud_cb,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/velodyne_points_map',      # 변환된 결과 토픽
            10
        )

        self.target_frame = 'map'        # 바꾸고 싶으면 여기만 수정
        self.get_logger().info('velodyne_to_map node ready')

    # -------------------------------------------------------------------------

    def pointcloud_cb(self, msg: PointCloud2):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, Time())
            cloud_map = tf2_sensor_msgs.do_transform_cloud(msg, tf)
            cloud_map.header.frame_id = 'map'
            cloud_map.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(cloud_map)
        except Exception as e:
            self.get_logger().warn(f'Unable to transform cloud: {e}')

# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = VelodyneToMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
