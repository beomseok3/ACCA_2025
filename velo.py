#!/usr/bin/env python3
# velodyne_stamp_fix.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
class VelodyneStampFix(Node):
    def __init__(self):
        super().__init__('velodyne_stamp_fix')

        # ▶ 입력: 원본 LiDAR
        self.sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',          # 필요하면 수정
            self.callback,
            100
        )
        self.create_subscription(TFMessage,
            '/tf',                      # 필요하면 수정
            self.time,
            100
        )

        # ▶ 출력: 타임스탬프 갱신 버전
        self.pub = self.create_publisher(
            PointCloud2,
            '/velodyne_points_latest',   # RViz에서는 이 토픽을 표시
            100
        )

        self.get_logger().info('velodyne_stamp_fix node ready')
        self.time = None

    def callback(self, msg: PointCloud2):
        # ① 현재 시간으로 헤더 stamp 교체
        if self.time is None:
            self.get_logger().warn('velodyne_stamp_fix: time is None')
            return
        msg.header.stamp = self.time
        # ② 그대로 퍼블리시
        self.pub.publish(msg)
    
    def time(self, msg: TFMessage):
        # ① 현재 시간으로 헤더 stamp 교체
        self.time = msg.transforms[0].header.stamp
        print(self.time)

def main(args=None):
    rclpy.init(args=args)
    node = VelodyneStampFix()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
