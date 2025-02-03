import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs_py.point_cloud2 import create_cloud

class ReflectivityFilter(Node):
    def __init__(self):
        super().__init__('reflectivity_filter')

        # ROS 2 구독 & 퍼블리셔 설정
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cropbox_filtered',  # Velodyne 포인트 클라우드 토픽
            self.pointcloud_callback,
            10)
        
        self.publisher = self.create_publisher(PointCloud2, '/filtered_pointcloud', 10)

        self.threshold = 90  # Intensity 필터링 임계값 (조정 가능)

    def pointcloud_callback(self, msg):
         # Velodyne PointCloud2 메시지 읽기 (x, y, z, intensity 필드)
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True).tolist())

        # 데이터가 없으면 처리하지 않음
        if len(points_list) == 0:
            print("Received empty point cloud!")
            return
        print(points_list)

        # numpy 배열로 변환 (각 포인트는 (x, y, z, intensity) 튜플임)
        points = np.array(points_list)
        print(points)
        # intensity 값이 threshold 이상인 포인트만 필터링
        threshold = self.threshold  # 예시로 intensity threshold 설정
        filtered_points = points[points[:, 3] > threshold]
        print(filtered_points)
        print(f"Filtered {len(filtered_points)} points out of {len(points)}")

        # 필터링된 포인트를 다시 PointCloud2 메시지로 변환해서 퍼블리시 가능
        # ...
        # 새로운 PointCloud2 메시지 생성
        filtered_msg = create_cloud(
            msg.header,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ],
            points=filtered_points
        )

        # 필터링된 포인트 클라우드 퍼블리시
        self.publisher.publish(filtered_msg)
        self.get_logger().info(f"Published filtered PointCloud2 with {filtered_points.shape[0]} points")

def main(args=None):
    rclpy.init(args=args)
    node = ReflectivityFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
