import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/filtered_pointcloud',
            self.pointcloud_callback,
            10
        )
        self.get_logger().info('Lane Detection Node started')
        
    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array
        cloud = self.convert_pointcloud2_to_numpy(msg)
        if cloud is None:
            self.get_logger().warn('Empty PointCloud received')
            return
        
        # Extract intensity values
        intensity = cloud[:, 3]  # Assuming intensity is the 4th column
        
        # Compute histogram
        hist_bin_resolution = 0.2
        hist_vals, bin_edges = np.histogram(intensity, bins=int(np.ptp(intensity) / hist_bin_resolution))
        bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
        
        # Detect peaks
        peaks, _ = find_peaks(hist_vals, height=np.mean(hist_vals))
        start_points = bin_centers[peaks]
        
        # Visualize histogram and peaks
        plt.figure()
        plt.plot(bin_centers, hist_vals, '--k', label='Histogram')
        plt.plot(start_points, hist_vals[peaks], 'ro', label='Detected Peaks')
        plt.legend()
        plt.show()

        # Initialize sliding windows
        lane_width = 4.0
        self.sliding_window(cloud, start_points, lane_width)

    def sliding_window(self, cloud, start_points, lane_width):
        # Example for sliding window initialization
        h_bin_res = 0.8
        v_bin_res = 1.0
        
        lane_points = []
        for start_y in start_points:
            current_window = self.create_window(cloud, start_y, h_bin_res, v_bin_res)
            lane_points.append(current_window)
        
        # Visualize results
        self.visualize_lane_points(cloud, lane_points)

    def create_window(self, cloud, start_y, h_bin_res, v_bin_res):
        # Create a sliding window and return points within it
        x_range = cloud[:, 0]
        y_range = cloud[:, 1]
        
        mask = (np.abs(y_range - start_y) < h_bin_res / 2)
        return cloud[mask]

    def visualize_lane_points(self, cloud, lane_points):
        # Visualize lane points using Open3D
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(cloud[:, :3])
        
        for lane in lane_points:
            lane_pc = o3d.geometry.PointCloud()
            lane_pc.points = o3d.utility.Vector3dVector(lane[:, :3])
            lane_pc.paint_uniform_color([1, 0, 0])  # Red for lane points
            pc += lane_pc
        
        o3d.visualization.draw_geometries([pc])

    def convert_pointcloud2_to_numpy(self, cloud_msg):
        points_list = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True).tolist())
        return np.array(points_list)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
