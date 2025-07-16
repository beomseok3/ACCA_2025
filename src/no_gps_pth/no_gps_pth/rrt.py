import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import math
import random
import os
import sys
from erp42_msgs.msg import ControlMessage, SerialFeedBack
from scipy.interpolate import CubicSpline
import numpy as np

# 현재 파일의 디렉토리 경로
current_dir = os.path.dirname(os.path.abspath(__file__))

# 프로젝트 루트 경로 추가
project_root = os.path.abspath(os.path.join("/home/acca/acca_ws/src/no_gps_pth/no_gps_pth"))
sys.path.append(project_root)

from stanley import Stanley

class State:
    def __init__(self):
        self.x = 0.0    # x 좌표
        self.y = 0.0    # y 좌표
        self.yaw = 0.0  # 방향(라디안)
        self.v = 0.0    # 속도

# RRT* 노드 구조체
class RRTNode:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0.0  # 부모 노드까지의 비용

class RRTStarPlanner(Node):
    # RRT* 경로 계획 노드
    # 장애물 회피 및 최적 경로 탐색을 위한 RRT* 알고리즘 구현
    # 이 노드는 ROS2 노드로, 장애물 맵을 구독하고 경로를 발행합니다.
    
    def __init__(self):
        super().__init__('rrt_star_planner')
        
        #
        self.stanley = Stanley()
        self.state = State()
        # 파라미터
        self.last_target_idx = 0
        self.map_width = 300
        self.map_height = 100
        self.resolution = 0.1
        self.obstacle_cost = 100
        self.max_iter = 500
        self.step_size = 0.5
        self.goal_radius = 1.0
        self.near_radius = 2.0
        self.x_min = -15.0
        self.x_max = 15.0
        self.y_min = -5.0
        self.y_max = 5.0

        # 맵 정보
        self.costmap = None
        self.origin = (0.0, 0.0)
        self.map_ready = False

        # ROS 통신
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/odom_costmap', self.costmap_callback, 10)
        self.path_pub = self.create_publisher(Path, '/rrt_star_path', 10)
        self.feedback_sub = self.create_subscription(SerialFeedBack, '/erp42_feedback', self.feedback_callback, 10)
        self.cmd_pub = self.create_publisher(ControlMessage, "cmd_msg", 10)
        ...
        # Pure Pursuit용 경로 저장
        self.planned_path_x_list = []
        self.planned_path_y_list = []
        self.create_timer(0.033, self.control_callback)

        # 목표점 (예시: 맵 우측 중앙)
        self.goal = (14,0)

        # 주기적 경로계획 (2Hz)
        self.create_timer(0.033, self.timer_callback)
        
    def feedback_callback(self, msg):
        # 위치/자세 정보는 없으므로, 속도와 steer만 사용
        self.state.v = msg.speed
        self.state.yaw = 0 #msg.steer 
        
    def control_callback(self):
        if len(self.planned_path_x_list) > 1 and self.map_ready:   
            msg = ControlMessage()
            delta, current_target_idx, _,_,_,_ = self.stanley.stanley_control(
                self.state,
                self.planned_path_x_list,
                self.planned_path_y_list,
                self.planned_path_yaw_list,
                self.last_target_idx
            )
            self.last_target_idx = current_target_idx
            # delta는 라디안, msg.steer도 라디안으로 전달
            msg.steer = -int(math.degrees(delta))  # 라디안 → 도 단위로 변환
            msg.speed = 30
            msg.gear = 2
            msg.estop = 0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f'제어 명령 발행: steer={delta:.3f} rad, speed=30')


    def costmap_callback(self, msg):
        # 코스트맵 메시지 수신
        self.costmap = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_ready = True

    def timer_callback(self):
        if not self.map_ready:
            return
        path = self.plan_rrt_star()
        if path:
            self.publish_path(path)

    def plan_rrt_star(self):
        # 시작점: (0,0) (맵 좌측 하단)
        start = (0.0, 0.0)
        goal = self.goal

        tree = [RRTNode(start[0], start[1])]
        for _ in range(self.max_iter):
            # 1. 샘플링 (목표점 bias 10%)
            if random.random() < 0.1:
                sample = goal
            else:
                sample = self.sample_free()
            # 2. 트리에서 가장 가까운 노드 찾기
            nearest = min(tree, key=lambda n: self.dist(n, sample))
            # 3. 확장
            new_node = self.steer(nearest, sample)
            if self.edge_collision(nearest, new_node):
                continue
            # 4. 근방 노드 찾기
            near_nodes = [n for n in tree if self.dist(n, (new_node.x, new_node.y)) < self.near_radius]
            # 5. 최적 부모 선택
            min_cost = nearest.cost + self.dist(nearest, (new_node.x, new_node.y))
            best_parent = nearest
            for n in near_nodes:
                cost = n.cost + self.dist(n, (new_node.x, new_node.y))
                if cost < min_cost and not self.edge_collision(n, new_node):
                    min_cost = cost
                    best_parent = n
            new_node.parent = best_parent
            new_node.cost = min_cost
            tree.append(new_node)
            # 6. Rewiring
            for n in near_nodes:
                cost_through_new = new_node.cost + self.dist(new_node, (n.x, n.y))
                if cost_through_new < n.cost and not self.edge_collision(new_node, n):
                    n.parent = new_node
                    n.cost = cost_through_new
            # 7. 목표 도달 체크
            if self.dist(new_node, goal) < self.goal_radius:
                return self.extract_path(new_node)
        return None

    def sample_free(self):
        while True:
            # 월드 좌표계 범위에서 샘플링
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            if not self.is_collision(x, y):
                return (x, y)


    def is_collision(self, x, y):
        # 월드 좌표 → 그리드 좌표 변환 (origin 고려)
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        
        if gx < 0 or gx >= self.map_width or gy < 0 or gy >= self.map_height:
            return True
        return self.costmap[gy, gx] >= self.obstacle_cost

    def edge_collision(self, node1, node2):
        # node1, node2: RRTNode
        dist = self.dist(node1, (node2.x, node2.y))
        steps = int(dist / (self.resolution * 0.5))
        for i in range(steps + 1):
            ratio = i / max(steps, 1)
            x = node1.x * (1 - ratio) + node2.x * ratio
            y = node1.y * (1 - ratio) + node2.y * ratio
            if self.is_collision(x, y):
                return True
        return False

    def steer(self, from_node, to_point):
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return RRTNode(from_node.x, from_node.y, from_node)
        step = min(self.step_size, dist)
        x = from_node.x + dx / dist * step
        y = from_node.y + dy / dist * step
        return RRTNode(x, y)

    def dist(self, node, point):
        return math.hypot(node.x - point[0], node.y - point[1])

    def extract_path(self, node):
        path = []
        cx, cy = [], []
        while node is not None:
            path.append((node.x, node.y))
            cx.append(node.x)
            cy.append(node.y)
            node = node.parent
        path.reverse()
        cx.reverse()
        cy.reverse()
        # 스플라인 보간 적용
        x_spline, y_spline = self.spline_interpolate_path(cx, cy, num_points=100)
        self.planned_path_x_list = x_spline.tolist()
        self.planned_path_y_list = y_spline.tolist()
        self.planned_path_yaw_list = self.compute_yaw_list(self.planned_path_x_list, self.planned_path_y_list)
        return list(zip(x_spline, y_spline))
    
    def spline_interpolate_path(self,cx, cy, num_points=100):
        """
        RRT* 경로 점(cx, cy)을 Cubic Spline으로 보간해 더 촘촘하고 부드러운 경로 반환
        """
        # 원본 경로 점 인덱스
        t = np.arange(len(cx))
        # 보간할 인덱스 (더 촘촘히)
        t_new = np.linspace(0, len(cx) - 1, num_points)
        # Cubic Spline 보간 함수 생성
        cs_x = CubicSpline(t, cx, bc_type='natural')
        cs_y = CubicSpline(t, cy, bc_type='natural')
        # 보간된 경로 좌표
        x_spline = cs_x(t_new)
        y_spline = cs_y(t_new)
        return x_spline, y_spline

    def compute_yaw_list(self, cx, cy):
        yaw_list = []
        for i in range(len(cx) - 1):
            dx = cx[i+1] - cx[i]
            dy = cy[i+1] - cy[i]
            yaw = math.atan2(dy, dx)
            yaw_list.append(yaw)
        if len(yaw_list) > 0:
            yaw_list.append(yaw_list[-1])
        else:
            yaw_list.append(0.0)
        return yaw_list


 
    
    def compute_yaw_list(self, cx, cy):
        """
        cx, cy : 경로의 x, y 좌표 리스트
        return : 각 점에서의 yaw(heading) 리스트 (라디안)
        """
        yaw_list = []
        for i in range(len(cx) - 1):
            dx = cx[i+1] - cx[i]
            dy = cy[i+1] - cy[i]
            yaw = math.atan2(dy, dx)
            yaw_list.append(yaw)
        # 마지막 점은 바로 앞 yaw와 동일하게 처리
        if len(yaw_list) > 0:
            yaw_list.append(yaw_list[-1])
        else:
            yaw_list.append(0.0)
        return yaw_list

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'velodyne'
        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = Point(x=x, y=y)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'RRT* 경로 발행: {len(path)} points')
        
        

def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()