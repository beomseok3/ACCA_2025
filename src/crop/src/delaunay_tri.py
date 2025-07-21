import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

class LaneDelaunayNode(Node):
    def __init__(self):
        super().__init__('lane_delaunay_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            'lane_filtered',
            self.lane_callback,
            10)
        self.get_logger().info("Listening to /lane_filtered topic...")

    def lane_callback(self, msg):
        # PointCloud2 메시지에서 (x, y) 좌표를 추출
        points = self.pointcloud2_to_numpy(msg)

        if len(points) < 3:
            self.get_logger().warn("Not enough points for Delaunay triangulation.")
            return

        # Delaunay Triangulation 수행
        tri = Delaunay(points)

        # 결과 출력
        self.visualize(points, tri)

    def pointcloud2_to_numpy(self, cloud_msg):
        """
        PointCloud2 메시지를 numpy 배열로 변환하여 (x, y) 좌표만 반환
        """
        point_list = []
        field_names = [field.name for field in cloud_msg.fields]

        if 'x' not in field_names or 'y' not in field_names:
            self.get_logger().error("PointCloud2 message does not contain 'x' and 'y' fields.")
            return np.array([])

        # raw data 읽기
        data = cloud_msg.data
        point_step = cloud_msg.point_step
        for i in range(cloud_msg.width * cloud_msg.height):
            offset = i * point_step
            x, y, _ = struct.unpack_from('fff', data, offset)
            point_list.append([x, y])

        return np.array(point_list)

    def visualize(self, points, tri):
        """
        Delaunay Triangulation 결과를 시각화
        """
        plt.figure()
        plt.triplot(points[:, 0], points[:, 1], tri.simplices, color='blue')
        plt.scatter(points[:, 0], points[:, 1], color='red', marker='o')
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.title("Delaunay Triangulation of Lane Points")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = LaneDelaunayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
