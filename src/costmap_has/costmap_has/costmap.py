import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import time
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid

class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')
        
        # 파라미터 설정
        self.declare_parameter('map_width', 300)
        self.declare_parameter('map_height', 300)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('obstacle_radius', 0.4)
        self.declare_parameter('obstacle_intensity', 255)  # 새로운 파라미터
        self.declare_parameter('decay_rate', 0.01)
        self.declare_parameter('parking_x',-12.0)
        self.declare_parameter('parking_y', -45.0)
        
        self.width = self.get_parameter('map_width').value
        self.height = self.get_parameter('map_height').value
        self.resolution = self.get_parameter('map_resolution').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.obstacle_intensity = self.get_parameter('obstacle_intensity').value
        self.decay_rate = self.get_parameter('decay_rate').value
        
        # Costmap 초기화
        self.costmap = np.zeros((self.height, self.width), dtype=np.uint8)
        self.last_update_time = time.time()
        
        # 구독자 설정
        self.pose_sub = self.create_subscription(
            PoseArray,
            'cone_poses_map',
            self.pose_callback,
            10)
        
        # 발행자 설정
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'costmap', 10)
        
        # 타이머 설정 (10Hz로 Costmap 업데이트 및 발행)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        self.update_costmap(msg.poses)


    def timer_callback(self):
        self.decay_costmap()
        self.publish_costmap()

    def update_costmap(self, poses):
        temp_costmap = np.zeros_like(self.costmap, dtype=np.float32)
        for pose in poses:
            x, y = int((pose.position.x-self.get_parameter("parking_x").value) / self.resolution), int((pose.position.y-self.get_parameter("parking_y").value) / self.resolution)
            
            # 장애물 중심으로부터의 거리에 따라 강도를 조절하는 마스크 생성
            mask = np.zeros_like(temp_costmap, dtype=np.float32)
            cv2.circle(mask, (x, y), int(self.obstacle_radius / self.resolution), 1.0, -1)
            
            # 가우시안 블러를 적용하여 부드러운 그라데이션 효과 생성
            mask = cv2.GaussianBlur(mask, (15, 15), 0)
            
            # 장애물 강도를 적용
            mask *= self.obstacle_intensity
            
            # 기존 costmap과 새로운 정보를 결합 (최대값 사용)
            temp_costmap = np.maximum(temp_costmap, mask)
        
        # 최종 결과를 uint8로 변환
        self.costmap = np.maximum(self.costmap, temp_costmap).astype(np.uint8)

    def decay_costmap(self):
        current_time = time.time()
        decay_factor = np.exp(-self.decay_rate * (current_time - self.last_update_time))
        self.costmap = (self.costmap * decay_factor).astype(np.uint8)
        self.last_update_time = current_time

    def publish_costmap(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.get_parameter('parking_x').value
        msg.info.origin.position.y = self.get_parameter('parking_y').value
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = (self.costmap / 255.0 * 100).flatten().astype(int).tolist()
        self.costmap_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
