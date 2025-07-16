import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class AccumulatingCostmap(Node):
    def __init__(self):
        super().__init__('accumulating_costmap')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_width', 300),
                ('map_height', 100),
                ('resolution', 0.1),
                ('obstacle_radius', 1.0),
                ('lane_radius', 1),
                ('obstacle_cost', 80),
                ('lane_cost', 80),
                ('max_cost', 100)
            ]
        )

        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.resolution = self.get_parameter('resolution').value

        self.lane_sub = self.create_subscription(
            PointCloud2, '/lane_filtered', self.lane_callback, 10)
        self.obstacle_sub = self.create_subscription(
            PoseArray, '/cone_poses', self.obstacle_callback, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/odom_costmap', 10)

        self.lane_points = None
        self.obstacle_poses = None

        # 10Hz로 코스트맵 생성 및 발행
        self.create_timer(0.1, self.generate_costmap)

    def relative_to_grid(self, x, y):
        """상대 좌표를 그리드 좌표로 변환"""
        center_x = self.map_width // 2
        center_y = self.map_height // 2
        gx = int(center_x + x / self.resolution)
        gy = int(center_y + y / self.resolution)
        return (
            np.clip(gx, 0, self.map_width - 1),
            np.clip(gy, 0, self.map_height - 1)
        )

    def generate_costmap(self):
        """새로운 코스트맵 생성"""
        self.costmap = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        self.add_lane_to_costmap()
        self.add_obstacles_to_costmap()
        
        self.publish_costmap()
        
    def add_circle_to_costmap(self, cx, cy, rad_px, base_cost):
        y, x = np.ogrid[-rad_px:rad_px+1, -rad_px:rad_px+1]
        dist = np.sqrt(x**2 + y**2)
        mask = (dist <= rad_px)
        gradient = base_cost * np.exp(-dist/rad_px)

        # costmap 슬라이싱 범위
        y_min = max(0, cy - rad_px)
        y_max = min(self.map_height, cy + rad_px + 1)
        x_min = max(0, cx - rad_px)
        x_max = min(self.map_width, cx + rad_px + 1)

        # gradient/mask 슬라이싱 범위
        gy_min = y_min - (cy - rad_px)
        gy_max = gy_min + (y_max - y_min)
        gx_min = x_min - (cx - rad_px)
        gx_max = gx_min + (x_max - x_min)

        self.costmap[y_min:y_max, x_min:x_max] += gradient[gy_min:gy_max, gx_min:gx_max] * mask[gy_min:gy_max, gx_min:gx_max]


    def add_lane_to_costmap(self):
        if self.lane_points is not None:
            radius = self.get_parameter('lane_radius').value
            base_cost = self.get_parameter('lane_cost').value
            rad_px = int(radius / self.resolution)

            for point in self.lane_points:
                cx, cy = self.relative_to_grid(point[0], point[1])
                self.add_circle_to_costmap(cx, cy, rad_px, base_cost)

    def add_obstacles_to_costmap(self):
        if self.obstacle_poses is not None:
            radius = self.get_parameter('obstacle_radius').value
            base_cost = self.get_parameter('obstacle_cost').value
            rad_px = int(radius / self.resolution)

            for pose in self.obstacle_poses.poses:
                cx, cy = self.relative_to_grid(pose.position.x, pose.position.y)
                self.add_circle_to_costmap(cx, cy, rad_px, base_cost)


    def lane_callback(self, msg):
        """최신 차선 위치 저장"""
        self.lane_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    def obstacle_callback(self, msg):
        """최신 장애물 위치 저장"""
        self.obstacle_poses = msg

    def publish_costmap(self):
        """코스트맵 발행"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'velodyne'  # 차량 기준 프레임

        msg.info.resolution = self.resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        # 원점을 맵의 중앙으로 설정
        msg.info.origin.position.x = -self.map_width * self.resolution / 2
        msg.info.origin.position.y = -self.map_height * self.resolution / 2
        msg.info.origin.position.z = 0.0

        max_cost = self.get_parameter('max_cost').value
        normalized = np.clip(self.costmap / max_cost * 100, 0, 100)
        msg.data = normalized.astype(np.int8).flatten().tolist()

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AccumulatingCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
