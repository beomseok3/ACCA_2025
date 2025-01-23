import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from scipy.interpolate import CubicSpline
from scipy.cluster.hierarchy import linkage, fcluster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from shapely.geometry import Point, Polygon
from geometry_msgs.msg import Point as ROSPoint
from stanley import Stanley
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import ControlMessage




import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from scipy.interpolate import CubicSpline
from scipy.cluster.hierarchy import linkage, fcluster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from shapely.geometry import Point, Polygon
from geometry_msgs.msg import Point as ROSPoint
from stanley import Stanley
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import ControlMessage
from scipy.ndimage import gaussian_filter1d
import time as t

class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.get_parameter("/speed_supporter/he_gain").value
        self.ce_gain = node.get_parameter("/speed_supporter/ce_gain").value

        self.he_thr = node.get_parameter("/speed_supporter/he_thr").value
        self.ce_thr = node.get_parameter("/speed_supporter/ce_thr").value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res




class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_obstacle", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_obstacle", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    def PIDControl(self, speed, desired_value):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, 4, 10))
    



class Obstacle:
    def __init__(self, node):

        self.node = node
        self.sub_marker = self.node.create_subscription(
            MarkerArray, "/markers", self.call_marker, 10
        )  # 장애물 위치

        self.globalpath_sub = self.node.create_subscription(
            Path,
            "global_path",
            self.callback_global,
            qos_profile=qos_profile_system_default,
        )
        # Publishers
        self.ref_path = self.node.create_publisher(Path, "/ref/path", 10)  # 1차선

        self.LocalPath_pub = self.node.create_publisher(
            Path, "/path/avoid_path", 10
        )  # 로컬 패스

        self.marker_pub = self.node.create_publisher(
            MarkerArray, "transformed_markers", 10
        )  # 곧 지울꺼

        self.polygon_marker_pub = self.node.create_publisher(
            Marker, "polygon_marker", 10
        )

        self.odometry = None

        self.yaw = None

        self.ori_x = None
        self.ori_y = None
        self.ori_x = None
        self.ori_w = None

        self.gx_list = []
        self.gy_list = []
        self.gyaw_list = []

        self.odom_pose = np.array([0.0, 0.0])
        self.odom_orientation = [0.0, 0.0, 0.0, 1.0]

        # 장애물 -> points_cloud
        self.obs = np.array([]).reshape(0, 2)  # 그냥 모든 장애물
        self.obstacle = []  # 라인내의 장애물
        self.near_obstacle = False

        self.obs_vector = None

        self.to_num = None

        # 도로 center_points #현재는 직접하는데 나중에 받아와야함
        self.ref_path_points = None

        # local_path
        self.local_points = None  # list

        self.local_x = []
        self.local_y = []
        self.local_yaw = []

        self.state = "static_avoid"

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)
        self.code_start = True
        self.once = True
        # Polygon points

        self.polygon_points = [
            (152.68667602539062, -76.75296783447266),
            (149.96263122558594, -73.62805938720703),
            (54.17646026611328, -135.12939453125),
            (52.68206787109375, -131.16880798339844),
        ]
        # Create the polygon
        self.polygon = Polygon(self.polygon_points)

    def callback_global(self, msg):
        for p in msg.poses:
            self.gx_list.append(p.pose.position.x)
            self.gy_list.append(p.pose.position.y)
            _, _, yaw = euler_from_quaternion(
                [
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ]
            )
            self.gyaw_list.append(yaw)

    def call_marker(self, msg):
        if self.code_start:
            if self.odometry is not None:
                markers = msg.markers
                transformed_marker_array = MarkerArray()
                observed_points = []

                for p in markers:
                    points = np.array([[point.x, point.y] for point in p.points])
                    if len(points) > 0:
                        # points의 중점을 계산
                        center = np.min(points, axis=0)

                        transformed_center = self.transform_cluster_centers(
                            np.array([center])
                        )

                        # 변환된 중심점을 저장
                        observed_points.append(transformed_center[0])

                        # 시각화용 마커 생성
                        transformed_marker = Marker()
                        transformed_marker.header.frame_id = "map"
                        transformed_marker.header.stamp = (
                            self.node.get_clock().now().to_msg()
                        )
                        transformed_marker.ns = "transformed_marker"
                        transformed_marker.id = p.id
                        transformed_marker.type = Marker.SPHERE
                        transformed_marker.action = Marker.ADD
                        transformed_marker.pose.position.x = transformed_center[0, 0]
                        transformed_marker.pose.position.y = transformed_center[0, 1]
                        transformed_marker.pose.position.z = 0.0
                        transformed_marker.scale.x = 0.5
                        transformed_marker.scale.y = 0.5
                        transformed_marker.scale.z = 0.5
                        transformed_marker.color.a = 1.0
                        transformed_marker.color.r = 0.0
                        transformed_marker.color.g = 1.0
                        transformed_marker.color.b = 0.0

                        transformed_marker_array.markers.append(transformed_marker)

                # 변환된 중심점들을 numpy 배열로 저장
                if observed_points:
                    self.obs = np.array(observed_points)

                # 변환된 마커 퍼블리시
                self.marker_pub.publish(transformed_marker_array)
            self.filter_obstacles_in_polygon(self.polygon_points)

    def rotate_points(self, points, angle):
        angle_radians = np.deg2rad(angle)
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), -np.sin(angle_radians)],
                [np.sin(angle_radians), np.cos(angle_radians)],
            ]
        )
        rotated_points = np.dot(points, rotation_matrix)
        return rotated_points

    def transform_cluster_centers(self, cluster_centers):
        if len(cluster_centers) == 0:
            return np.array([])
        euler = euler_from_quaternion(self.odom_orientation)
        _, _, yaw = euler

        rotation_angle = np.rad2deg(-yaw)

        rotated_centers = self.rotate_points(cluster_centers, rotation_angle)

        transformed_centers = rotated_centers + np.array(
            (self.odometry.x, self.odometry.y)
        )

        return transformed_centers

    def filter_obstacles_in_polygon(self, polygon_points):
        # Create a Polygon object from the given polygon points
        obstacle_polygon = Polygon(polygon_points)

        # Initialize or clear self.obstacle
        # self.obstacle = []

        # Iterate through the obstacles in self.obs
        for obs_x, obs_y in self.obs:
            # Create a Point object for each obstacle
            obstacle_point = Point(obs_x, obs_y)

            # Check if the obstacle is within the polygon
            if obstacle_polygon.contains(obstacle_point):
                # Add the obstacle to self.obstacle if it's inside the polygon
                self.obstacle.append((obs_x, obs_y))

    def publish_polygon_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "polygon"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to create polygon edges
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for point in self.polygon.exterior.coords:
            p = ROSPoint()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0  # z-coordinate is set to 0
            marker.points.append(p)

        # Close the polygon by adding the first point again
        if marker.points:
            marker.points.append(marker.points[0])

        # Publish the marker
        self.polygon_marker_pub.publish(marker)

    def publish_ref_path(self, wx, wy):
        # Interpolate y values given x using CubicSpline
        cs_x = CubicSpline(range(len(wx)), wx)
        cs_y = CubicSpline(range(len(wy)), wy)

        # Calculate total path length based on the given waypoints
        distances = np.sqrt(np.diff(wx) ** 2 + np.diff(wy) ** 2)
        total_length = np.sum(distances)

        # 간격
        sampling = 0.1
        # Sampling intervals for generating the path with 0.1 distance between points
        s = np.arange(0, len(wx) - 1, sampling / total_length * (len(wx) - 1))

        # Interpolated path coordinates
        rx = cs_x(s)
        ry = cs_y(s)

        # Save path and direction data
        path_points = np.vstack((rx, ry)).T

        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Compute orientation from current position
            yaw = np.arctan2(y - self.odometry.y, x - self.odometry.x)
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.append(pose)

        # Publish the global path
        self.ref_path.publish(path)

        # Save global path for later use
        self.ref_path_points = path_points

    def move_points_away_from_obstacles(
        self, distance_threshold=0.5, shift_distance=3
    ):

        # self.last_avoid_point 가있을때 그인덱스 이후 부터는 이동 안함
        # List to hold the indices of points that need to be moved
        indices_to_move = []

        for i, point in enumerate(self.ref_path_points):
            for obs_x, obs_y in self.obstacle:
                # Calculate the distance between the path point and the obstacle
                distance = sqrt((obs_x - point[0]) ** 2 + (obs_y - point[1]) ** 2)

                # If the path point is within the distance threshold from the obstacle
                if distance < distance_threshold:
                    indices_to_move.append(i)
                    break  # No need to check other obstacles for this point

        # Move points that are close to obstacles
        adjusted_path_points = self.ref_path_points[:]
        for i in indices_to_move:
            point = self.ref_path_points[i]
            for obs_x, obs_y in self.obstacle:
                # Calculate the distance between the path point and the obstacle
                distance = sqrt((obs_x - point[0]) ** 2 + (obs_y - point[1]) ** 2)

                # If the path point is within the distance threshold from the obstacle
                if distance < distance_threshold:
                    # Calculate the direction vector from the obstacle to the path point
                    direction_x = point[0] - obs_x
                    direction_y = point[1] - obs_y

                    # if direction_x   < 0:
                    #     direction_x = 0
                    #     direction_y = 0

                    # Normalize the direction vector
                    length = sqrt(direction_x**2 + direction_y**2)
                    if length > 0:
                        direction_x /= length
                        direction_y /= length

                        # Shift the point in the direction away from the obstacle
                        adjusted_x = point[0] + direction_x * shift_distance
                        adjusted_y = point[1] + direction_y * shift_distance

                        # Update the path with the adjusted point
                        adjusted_path_points[i] = (adjusted_x, adjusted_y)
                    break  # No need to check other obstacles for this point

        # Replace the original path points with the adjusted ones

        self.local_path = adjusted_path_points
        self.check_available_path()

    def check_available_path(self):
        if len(self.local_path) < 2:
            return  # Not enough points to interpolate

        # Extract x and y coordinates
        x_coords, y_coords = zip(*self.local_path)

        # Create cubic splines for x and y coordinates
        cs_x = CubicSpline(range(len(x_coords)), x_coords)
        cs_y = CubicSpline(range(len(y_coords)), y_coords)

        # Create a new set of points with a smaller sampling interval
        num_samples = 50  # Number of samples for smooth path
        sample_indices = np.linspace(0, len(x_coords) - 1, num_samples)
        smooth_x = cs_x(sample_indices)
        smooth_y = cs_y(sample_indices)

        # Smooth the path using Gaussian filter
        smooth_x = gaussian_filter1d(smooth_x, sigma=1)  # Adjust sigma for smoothing
        smooth_y = gaussian_filter1d(smooth_y, sigma=1)  # Adjust sigma for smoothing

        # Combine smoothed x and y coordinates
        smooth_path_points = np.vstack((smooth_x, smooth_y)).T

        # Update the local_path with smooth path points
        self.local_path = smooth_path_points.tolist()

        # Publish the adjusted path
        self.publish_adjusted_path()

    def publish_adjusted_path(self):
        if len(self.local_path) < 2:
            return  # Not enough points to interpolate

        # Extract x and y coordinates
        x_coords, y_coords = zip(*self.local_path)

        # Compute distances between points
        distances = np.sqrt(np.diff(x_coords) ** 2 + np.diff(y_coords) ** 2)
        total_length = np.sum(distances)

        sampling = 0.1
        # Number of points with 0.1 meter spacing
        num_points = int(total_length / sampling) + 1
        s = np.linspace(0, total_length, num_points)

        # Create cubic splines for x and y coordinates
        cs_x = CubicSpline(np.cumsum([0] + list(distances)), x_coords)
        cs_y = CubicSpline(np.cumsum([0] + list(distances)), y_coords)

        # Interpolated path coordinates
        smooth_x = cs_x(s)
        smooth_y = cs_y(s)

        # Combine x and y coordinates
        smooth_path_points = np.vstack((smooth_x, smooth_y)).T

        # Prepare the Path message
        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # List to store local path details
        local_x, local_y, local_yaw = [], [], []

        # Convert the smoothed path points to PoseStamped and compute orientation
        for i, (x, y) in enumerate(smooth_path_points):
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -1.0
            self.local_points.append((x, y))
            # Compute orientation from current position
            if i < len(smooth_path_points) - 1:
                next_x, next_y = smooth_path_points[i + 1]
                yaw = np.arctan2(next_y - y, next_x - x)
            else:
                # Use the same orientation as the previous point for the last point
                yaw = local_yaw[-1] if local_yaw else 0.0

            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.append(pose)
            local_x.append(x)
            local_y.append(y)
            local_yaw.append(yaw)

        # Publish the adjusted path
        self.LocalPath_pub.publish(path)

        # Save local path details
        self.local_x = local_x
        self.local_y = local_y
        self.local_yaw = local_yaw

    def control_obstacle(self, odometry):
        self.odometry = odometry
        self.timer_callback()
        msg = ControlMessage()

        if len(self.local_x) != 0:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                self.local_x,
                self.local_y,
                self.local_yaw,
                h_gain=1.0,
                c_gain=0.8,
            )
            target_speed = 5.0
            adapted_speed = self.ss.adaptSpeed(
                target_speed, hdr, ctr, min_value=4, max_value=6
            )
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
            brake = self.cacluate_brake(adapted_speed)
        else:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                self.gx_list,
                self.gy_list,
                self.gyaw_list,
                h_gain=1.0,
                c_gain=0.8,
            )
            target_speed = 7.0
            adapted_speed = self.ss.adaptSpeed(
                target_speed, hdr, ctr, min_value=6, max_value=8
            )
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
            brake = self.cacluate_brake(adapted_speed)

        # msg.speed = int(speed) * 10
        msg.speed = 30
        msg.steer = int(degrees((-1) * steer))
        msg.gear = 2
        msg.brake = int(brake)

        if self.target_idx >= len(self.local_points) - 10:

            self.code_start = False

            return msg, True
        else:
            return msg, False

    def cacluate_brake(
        self, adapted_speed
    ):  # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.odometry.v * 3.6 >= adapted_speed:
            brake = (abs(self.odometry.v * 3.6 - adapted_speed) / 20.0) * 200
        else:
            brake = 0
        return brake

    def stopTOmove(self):

        # 내 위치보다 앞에 장애물이 있으면 self.go.v = 0
        for obs_x, obs_y in self.obstacle:  ####중요 원래는  self.obstacle임
            _, _, yaw = euler_from_quaternion(self.odom_orientation)

            vehicle_direction = np.array([cos(yaw), sin(yaw)])

            obs_vector = np.array([obs_x - self.odometry.x, obs_y - self.odometry.y])
            obs_length = sqrt(
                (obs_x - self.odometry.x) ** 2 + (obs_y - self.odometry.y) ** 2
            )

            if np.dot(vehicle_direction, obs_vector) > 0 and obs_length < 15:
                # 장애물 벡터와 차량 방향 벡터 사이의 각도 계산
                angle = degrees(
                    atan2(obs_vector[1], obs_vector[0])
                    - atan2(vehicle_direction[1], vehicle_direction[0])
                )

                # 각도를 0~360 사이로 변환
                angle = (angle + 360) % 360

                # 각도가 전방 60도(±30도) 안에 들어오는지 확인
                if (
                    angle < 30 or angle > 330
                ):  # 30도 안쪽 또는 330도 이상 (반시계 방향 30도)
                    # print(np.dot(vehicle_direction, obs_vector), obs_length)
                    # print(obs_x, obs_y)

                    self.velocity = 0
                    print("정지정지정지")
                    t.sleep(5)
                    self.velocity = 1
                    self.state = None
                    print("출발")

                    return

    def check_proximity_to_line(self, px, py):
        # 라인의 두 점
        min_dis = 100
        max_dis = 0
        x1, y1 = self.polygon_points[0][0], self.polygon_points[0][1]
        x2, y2 = self.polygon_points[3][0], self.polygon_points[3][1]

        self.last_avoid_point = None

        for lx, ly in self.local_points:
            # 라인과 현재 위치 (px, py) 사이의 거리 계산
            area = abs((y2 - y1) * lx - (x2 - x1) * ly + (x2 * y1 - x1 * y2))
            AB = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
            distance = area / AB

            if min_dis > distance:
                min_dis = distance
            if max_dis < distance:
                max_dis = distance

            # 거리가 일정 임계값 이하이면 상태를 moving_obs로 설정
            threshold1 = 1.2  # 거리 임계값 (예시)
            threshold2 = 3.3

            if distance <= threshold1 or distance > threshold2:

                # print("stop")
                # self.state = "moving_obs"
                self.last_avoid_point = [lx, ly]
        # print(min_dis, max_dis)
        if self.last_avoid_point is not None:
            distance = (self.last_avoid_point[0] - px) ** 2 + (
                self.last_avoid_point[1] - py
            ) ** 2
            # print(distance)
            if distance < 2:

                self.state = "moving_obs"

    def timer_callback(self):
        if not self.code_start:
            self.code_start = True

        self.odom_pose = np.array([self.odometry.x, self.odometry.y])
        q = quaternion_from_euler(0, 0, self.odometry.yaw)
        self.odom_orientation = [q[0], q[1], q[2], q[3]]

        if self.odometry.x is not None:

            # k-city2 @작은거
            #  Position(180.184, -33.1158, 0),
            #  Position(73.5005, -93.9311, 0),
            # [INFO] [1728997268.410992466] [rviz2]: Setting estimate pose: Frame:map, Position(158.006, -44.7224, 0), Orientation(0, 0, 0, 1) = Angle: 0
            # [INFO] [1728997271.973089410] [rviz2]: Setting estimate pose: Frame:map, Position(146.041, -51.7367, 0), Orientation(0, 0, 0, 1) = Angle: 0
            if self.once:
                self.publish_ref_path(
                    wx=[
                        (self.polygon_points[0][0] + self.polygon_points[1][0]) / 2,
                        (self.polygon_points[2][0] + self.polygon_points[3][0]) / 2,
                    ],
                    wy=[
                        (self.polygon_points[0][1] + self.polygon_points[1][1]) / 2,
                        (self.polygon_points[2][1] + self.polygon_points[3][1]) / 2,
                    ],
                )  # 1차선 center line

                self.local_points = self.ref_path_points.tolist()
                self.once = False
        # self.publish_local_path(self.local_points)
        self.publish_polygon_marker()
        # self.move_points_away_from_obstacles()
        if self.state is not None:
            self.check_proximity_to_line(self.odometry.x, self.odometry.y)
            if self.state == "static_avoid":
                self.velocity = 1

                self.move_points_away_from_obstacles()
            if self.state == "moving_obs":

                self.stopTOmove()
            print(self.state)
