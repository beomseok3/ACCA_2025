import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

# astar 코드
import heapq
import math

# 라인 만들거
from scipy.interpolate import CubicSpline

###패스가 보가으로 만들고 delta yaw 가 입계치 이상이면 안된다


class AStar:
    def __init__(self, obs_map=[]):
        self.move_distance = 0.1  # 이동 반경
        self.obs_range = 1  # 장애물 반경

        self.start = (0, 0)
        self.goal = (0, 0)

        self.map = obs_map  # 장애물 맵
        self.vertical = abs(self.start[1] - self.goal[1])  # 맵 수직 길이
        self.horizontal = abs(self.start[0] - self.goal[0])  # 맵 수평 길이

        self.open_set = []  # heap 리스트
        self.came_from = {}  # path 경로 역 추적

    def heuristic(self, current, goal):  # 현재 위치에서 골까지 거리
        return math.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

    def reconstruct_path(self, current):
        """경로 역 추척."""
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append((current[0], current[1]))
        return path[::-1]

    def get_neighbors(self, current_pose):
        neighbors = []
        directions = [
            (-self.move_distance, 0),
            (self.move_distance, 0),
            (0, -self.move_distance),
            (0, self.move_distance),
            (-self.move_distance, -self.move_distance),
            (-self.move_distance, self.move_distance),
            (self.move_distance, -self.move_distance),
            (self.move_distance, self.move_distance),
        ]  # 좌, 우, 하, 상,,

        for d in directions:
            # 한칸 이동
            new_x = round(current_pose[0] + d[0], 1)
            new_y = round(current_pose[1] + d[1], 1)

            # 이동이 map 안에있고 장애물과 안 겹칠때
            if self.check_for_obstacle(current_pose):

                neighbors.append((new_x, new_y))
        return neighbors

    def check_for_obstacle(self, current_pose):
        # 장애물 위치 리스트 순회
        for obstacle in self.map:
            distance = self.heuristic(current_pose, obstacle)
            if distance <= self.obs_range:
                return False  # 장애물이 범위 내에 있음
            # if distance >= self.obs_range:
            #     return True  # 장애물이 범위 외에 있음
        return True  # 장애물 없음

    def is_valid_position(self, pose):
        # 시작점이나 목표점이 장애물과 너무 가까운지 확인
        for obstacle in self.map:
            if self.heuristic(pose, obstacle) <= self.obs_range:
                return False  # 너무 가까우면 유효하지 않음
        return True  # 유효한 위치

    def update_obstacle(self, obstacle_pose):
        self.map.append(obstacle_pose)
        return self.find_path()

    def update_goal(self, pose):
        self.goal = pose
        return self.find_path()

    def update_start(self, pose):
        self.start = pose
        return self.find_path()

    def find_path(self):

        if not self.is_valid_position(self.start):
            print("Starting position is too close to an obstacle.")
            return []

        if not self.is_valid_position(self.goal):
            print("Goal position is too close to an obstacle.")
            return []

        heapq.heappush(self.open_set, (0, self.start))  # (f값,현재위치)
        g_cost = {self.start: 0}  # 이동 코스트
        f_cost = {
            self.start: self.heuristic(self.start, self.goal)
        }  # 이동 코스트 + 휴리스틱 코스트

        while self.open_set:
            _, current = heapq.heappop(self.open_set)
            if self.heuristic(current, self.goal) <= 0.2:
                # 경로 역추적
                return self.reconstruct_path(current)

            for neighbor in self.get_neighbors(current):

                try:
                    new_g_cost = g_cost[current] + self.heuristic(current, neighbor)
                except KeyError:
                    # KeyError가 발생하면 해당 위치를 무시하고 계속 진행
                    continue
                if (
                    neighbor not in g_cost or new_g_cost < g_cost[neighbor]
                ):  # 지난곳이 아니거나 코스트가 낮아지면
                    self.came_from[neighbor] = current
                    g_cost[neighbor] = new_g_cost
                    f_cost[neighbor] = new_g_cost + self.heuristic(neighbor, self.goal)
                    heapq.heappush(self.open_set, (f_cost[neighbor], neighbor))

        return []


class AStarPathMaker(Node):
    def __init__(self):
        super().__init__("astar_path")

        self.sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.call_goal, 10
        )
        self.sub_obs = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.call_obs, 10
        )

        self.pub_obs = self.create_publisher(MarkerArray, "/obs_pose", 10)
        self.pub_global_path = self.create_publisher(Path, "/global_path", 10)
        self.pub_spline_path = self.create_publisher(Path, "/spline_path", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = (-5, -5)
        self.goal = (5, 5)

        self.obs = []

        self.astar = AStar()
        self.path = []
        self.marker_array = MarkerArray()
        self.marker_id = 0

    def call_goal(self, msg):
        self.goal = (round(msg.pose.position.x, 1), round(msg.pose.position.y, 1))
        self.path = self.astar.update_goal(self.goal)
        print("set_goal")

    def call_obs(self, msg):
        print("set_obs")
        self.obs.append(
            (round(msg.pose.pose.position.x, 1), round(msg.pose.pose.position.y, 1))
        )
        self.astar.update_obstacle(self.obs[-1])
        self.path = self.astar.update_goal(self.goal)

        # 시긱화
        marker = Marker()
        marker.header.frame_id = "map"  # RViz에서 사용할 프레임 설정
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacle"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 위치 설정
        marker.pose.position.x = msg.pose.pose.position.x
        marker.pose.position.y = msg.pose.pose.position.y
        marker.pose.position.z = msg.pose.pose.position.z
        marker.pose.orientation = msg.pose.pose.orientation

        # 크기 설정
        marker.scale.x = 0.5  # x 방향 크기
        marker.scale.y = 0.5  # y 방향 크기
        marker.scale.z = 0.5  # z 방향 크기

        # 색상 설정 (빨간색 구)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 불투명

        # MarkerArray에 추가
        self.marker_array.markers.append(marker)

        self.marker_id += 1

    def check_path(self):
        for point in self.path:
            for obs in self.obs:
                distance = math.sqrt(
                    (point[0] - obs[0]) ** 2 + (point[1] - obs[1]) ** 2
                )
                if distance <= 1:
                    return False
        return True

    def timer_callback(self):
        # 퍼블리시
        self.pub_obs.publish(self.marker_array)
        path = self.path  # self.astar.find_path()
        while not self.check_path():
            self.path = self.astar.update_goal(self.goal)
            print("not")
            path = self.astar.find_path()

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Path에 포인트 추가
        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])

            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        # Path 퍼블리시
        self.pub_global_path.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPathMaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
