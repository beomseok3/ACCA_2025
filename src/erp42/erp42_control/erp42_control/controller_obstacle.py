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


class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_obstacle", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_obstacle", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr_obstacle",0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr_obastacle",0.002).value

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
        return int(np.clip(self.speed, 4, 8))
    

class Obstacle:
    def __init__(self, node):

        self.node = node
        self.sub_marker = self.node.create_subscription(
            MarkerArray, "/markers", self.call_marker, 10
        )  # 장애물 위치

        # Publishers
        self.ref_path1 = self.node.create_publisher(Path, "/ref/path1", 10)  # 1차선
        self.ref_path2 = self.node.create_publisher(Path, "/ref/path2", 10)  # 2차선

        self.LocalPath_pub = self.node.create_publisher(
            Path, "/path/avoid_path", 10
        )  # 로컬 패스

        self.marker_pub = self.node.create_publisher(
            MarkerArray, "transformed_markers", 10
        )  # 곧 지울꺼

        self.road_poly = self.node.create_publisher(Marker, "visualization_marker", 10)
        self.odometry = None

        self.yaw = None

        self.ori_x = None
        self.ori_y = None
        self.ori_x = None
        self.ori_w = None


        self.odom_pose = np.array([0.0, 0.0])
        self.odom_orientation = [0.0, 0.0, 0.0, 1.0]

        # 장애물 -> points_cloud
        self.obs = np.array([]).reshape(0, 2)
        self.near_obstacle = False
        self.num1_obs = []
        self.num2_obs = []
        self.obs_vector = None

        self.to_num = None

        # 도로 center_points #현재는 직접하는데 나중에 받아와야함
        self.ref_path_points1 = None
        self.ref_path_points2 = None

        # local_path
        self.local_points = None  # list

        self.local_x = []
        self.local_y = []
        self.local_yaw = []

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.code_start = True


    def call_marker(self, msg):
        if self.code_start:
            # odometry가 유효한지 먼저 확인
            if (
                self.odometry is not None
                and self.odometry.x is not None
                and self.odometry.y is not None
            ):
                # msg와 msg.markers가 None이 아닌지 확인
                if msg is not None and hasattr(msg, "markers") and msg.markers:
                    markers = msg.markers
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

                    # 변환된 중심점들을 numpy 배열로 저장
                    if observed_points:
                        self.obs = np.array(observed_points)

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

    def obs_marker(self):
        # Marker array for visualization
        marker_array = MarkerArray()

        # Visualizing self.num1_obs points in green
        for idx, point in enumerate(self.num1_obs):
            green_marker = Marker()
            green_marker.header.frame_id = "map"
            green_marker.header.stamp = self.node.get_clock().now().to_msg()
            green_marker.ns = "num1_obs_markers"
            green_marker.id = idx  # Unique ID for each marker
            green_marker.type = Marker.SPHERE
            green_marker.action = Marker.ADD
            green_marker.pose.position.x = point[0]
            green_marker.pose.position.y = point[1]
            green_marker.pose.position.z = 0.0
            green_marker.scale.x = 0.5
            green_marker.scale.y = 0.5
            green_marker.scale.z = 0.5
            green_marker.color.a = 1.0  # Opacity
            green_marker.color.r = 0.0
            green_marker.color.g = 1.0
            green_marker.color.b = 0.0

            marker_array.markers.append(green_marker)

        # Visualizing self.num2_obs points in red
        for idx, point in enumerate(self.num2_obs):
            red_marker = Marker()
            red_marker.header.frame_id = "map"
            red_marker.header.stamp = self.node.get_clock().now().to_msg()
            red_marker.ns = "num2_obs_markers"
            red_marker.id = len(self.num1_obs) + idx  # Unique ID for each marker
            red_marker.type = Marker.SPHERE
            red_marker.action = Marker.ADD
            red_marker.pose.position.x = point[0]
            red_marker.pose.position.y = point[1]
            red_marker.pose.position.z = 0.0
            red_marker.scale.x = 0.5
            red_marker.scale.y = 0.5
            red_marker.scale.z = 0.5
            red_marker.color.a = 1.0  # Opacity
            red_marker.color.r = 1.0
            red_marker.color.g = 0.0
            red_marker.color.b = 0.0

            marker_array.markers.append(red_marker)

        # Publish the marker array
        self.marker_pub.publish(marker_array)

    def organize_obstacle_lists(self):
        # Calculate the current position as a numpy array
        current_position = np.array([self.odometry.x, self.odometry.y])

        # Calculate distances from the current position for num1_obs
        distances_num1 = [
            np.linalg.norm(np.array(point) - current_position)
            for point in self.num1_obs
        ]

        # Initialize variables to find the farthest point in num1_obs that is closest to num2_obs
        best_point = None
        best_distance = -1
        best_closest_distance = float("inf")

        # Check each point in num1_obs
        for i, point1 in enumerate(self.num1_obs):
            distance_from_current = distances_num1[i]

            # Calculate distances from point1 to all points in num2_obs
            closest_distance = min(
                np.linalg.norm(np.array(point1) - np.array(point2))
                for point2 in self.num2_obs
            )

            # Update the best point if it is farther from the current position and closer to any point in num2_obs
            if closest_distance < best_closest_distance or (
                closest_distance == best_closest_distance
                and distance_from_current > best_distance
            ):
                best_point = point1
                best_distance = distance_from_current
                best_closest_distance = closest_distance

        # Update num1_obs to contain the best point
        if best_point is not None:
            self.num1_obs = [best_point]  # Keep only the best point

        # Keep the three closest points in num2_obs
        if self.num2_obs:  # Ensure there's at least one point
            distances_num2 = [
                np.linalg.norm(np.array(point) - current_position)
                for point in self.num2_obs
            ]
            closest_indices = np.argsort(distances_num2)[
                :3
            ]  # Get indices of the three closest points
            self.num2_obs = [
                self.num2_obs[i] for i in closest_indices
            ]  # Keep only the closest three points

    def publish_ref_path(self, wx, wy, num=None):
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

        if num == 1:
            # Publish the global path
            self.ref_path1.publish(path)

            # Save global path for later use
            self.ref_path_points1 = path_points

        if num == 2:
            # Publish the global path
            self.ref_path2.publish(path)

            # Save global path for later use
            self.ref_path_points2 = path_points

    def publish_polygon(self, points, ns, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Set the scale of the lines (width)
        marker.scale.x = 0.1

        # Set color
        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        # Create the points for the polygon
        for point in points:
            p = ROSPoint()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0  # Assume flat polygon on ground plane
            marker.points.append(p)

        # Close the polygon by adding the first point again
        first_point = ROSPoint()
        first_point.x = points[0][0]
        first_point.y = points[0][1]
        first_point.z = 0.0
        marker.points.append(first_point)

        # Publish the marker
        self.road_poly.publish(marker)

    def publish_local_path(self, points):
        # Initialize lists to store x, y, and yaw values
        local_x = []
        local_y = []
        local_yaw = []

        # Extract x and y coordinates from points
        for x_points, y_points in points:
            local_x.append(x_points)
            local_y.append(y_points)

        # Calculate distances between consecutive points
        dx = np.diff(local_x)
        dy = np.diff(local_y)
        distances = np.sqrt(dx**2 + dy**2)

        # Interpolation to achieve 0.1 spacing
        total_distance = np.cumsum(distances)
        total_distance = np.insert(total_distance, 0, 0)  # Insert 0 at the beginning
        interp_distances = np.arange(
            0, total_distance[-1], 0.1
        )  # Interpolated distances

        # Interpolate x and y coordinates
        interp_x = np.interp(interp_distances, total_distance, local_x)
        interp_y = np.interp(interp_distances, total_distance, local_y)

        path_points = []

        path = Path()
        path.header.frame_id = "map"

        for i in range(len(interp_x) - 1):
            x = interp_x[i]
            y = interp_y[i]
            next_x = interp_x[i + 1]
            next_y = interp_y[i + 1]

            # Calculate yaw from the direction of the next point
            yaw = np.arctan2(next_y - y, next_x - x)
            local_yaw.append(yaw)

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -1.0  # Set z-coordinate

            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

            path_points.append((x, y))

        self.LocalPath_pub.publish(path)

        # Store the interpolated path points and yaw
        self.local_x = interp_x.tolist()
        self.local_y = interp_y.tolist()
        self.local_yaw = local_yaw

    def line_change(self):
        if len(self.num1_obs) > 0:
            for obs_x, obs_y in self.num1_obs:
                _, _, yaw = euler_from_quaternion(self.odom_orientation)

                # Compute the direction vector of the vehicle
                vehicle_direction = np.array([cos(yaw), sin(yaw)])
                # odom과 self.num1_obs 비교후 odom보다 앞에있으면 to_num = 2
                self.obs_vector = np.array(
                    [obs_x - self.odometry.x, obs_y - self.odometry.y]
                )

        if len(self.num2_obs) > 1:
            for obs_x, obs_y in self.num2_obs:
                _, _, yaw = euler_from_quaternion(self.odom_orientation)

                vehicle_direction = np.array([cos(yaw), sin(yaw)])

                obs_vector = np.array(
                    [obs_x - self.odometry.x, obs_y - self.odometry.y]
                )
                obs_length = sqrt(
                    (obs_x - self.odometry.x) ** 2 + (obs_y - self.odometry.y) ** 2
                )
                print(obs_length)
                if len(self.num1_obs) < 1:

                    if obs_length <= 3:  # 3m -> 2m -> 2.5로 변경 (1012)
                        print("긴급회피")
                        self.to_num = 1
                        self.local_points = self.ref_path_points1.tolist()
                        self.publish_local_path(self.local_points)
                        return

                else:
                    angle = degrees(
                        atan2(self.obs_vector[1], self.obs_vector[0])
                        - atan2(vehicle_direction[1], vehicle_direction[0])
                    )

                    # if angle < 0:
                    #     angle = angle + 360
                    # if (
                    #     np.dot(vehicle_direction, obs_vector) > 0
                    #     and (angle > 50 and angle < 70)
                    #     and obs_length < 5
                    # ):

                    if obs_length <= 3:  # 3m
                        print("회피")
                        self.to_num = 1
                        self.local_points = self.ref_path_points1.tolist()
                        self.publish_local_path(self.local_points)
                        return

        if self.to_num is None:
            self.local_points = self.ref_path_points2.tolist()
        self.to_num = 2

        self.publish_local_path(self.local_points)

    def check_obstacle(self, size_type):

        if size_type == "small":

            line1_DA = [
                (-37.1543, -55.9792),
                (-36.4105, -59.4389),
                (-3.86779, -62.9776),
                (-4.22124, -61.1396),
            ]  # DA = detection_area

            line2_DA = [
                (35.5591, 104.309),
                (35.9856, 105.058),
                (22.7638, 112.018),
                (22.2762, 110.953),
            ]
        if size_type == "big":
            line1_DA = [
                (-37.1543, -55.9792),
                (-36.4105, -59.4389),
                (-3.86779, -62.9776),
                (-4.22124, -61.1396),
            ]  # DA = detection_area
            line2_DA = [
                (132.002, 254.289),
                (133.863, 253.742),
                (138.002, 267.316),
                (135.92, 268.411),
            ]

        # Create polygons from the detection areas
        polygon1 = Polygon(line1_DA)
        polygon2 = Polygon(line2_DA)

        # Publish the first polygon
        self.publish_polygon(line1_DA, "line1", 0.0, 1.0, 0.0, 1.0)  # Green
        # Publish the second polygon
        self.publish_polygon(line2_DA, "line2", 1.0, 0.0, 0.0, 1.0)  # Red

        # Check if any of the obstacles are within the detection area

        for obs_point in self.obs:

            point = Point(obs_point)
            if polygon1.contains(point):

                self.num1_obs.append((point.x, point.y))
                break

        # Check if any of the obstacles are within the detection area
        for obs_point in self.obs:

            point = Point(obs_point)
            if polygon2.contains(point):

                self.num2_obs.append((point.x, point.y))
                break

    def control_obstacle(self, odometry, path):
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

        else:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path.cx,
                path.cy,
                path.cyaw,
                h_gain=0.5,
                c_gain=0.24,
            )
            target_speed = 7.0
            adapted_speed = self.ss.adaptSpeed(
                target_speed, hdr, ctr, min_value=6, max_value=8
            )
            speed = self.pid.PIDControl(self.odometry.v * 3.6, adapted_speed)

        msg.speed = int(speed) * 10
        msg.steer = int(degrees((-1) * steer))
        msg.gear = 2

        if self.target_idx >= len(self.local_points) - 10:
            self.to_num = None
            self.code_start = False
            self.num1_obs = []
            self.num2_obs = []
            return msg, True
        else:
            return msg, False



    def timer_callback(self):
        if not self.code_start:
            self.code_start = True

        self.odom_pose = np.array([self.odometry.x, self.odometry.y])
        q = quaternion_from_euler(0, 0, self.odometry.yaw)
        self.odom_orientation = [q[0], q[1], q[2], q[3]]

        if (
            self.odometry is not None
            and self.odometry.x is not None
            and self.odometry.y is not None
        ):
            if (self.odometry.x - 38.96862971336143) ** 2 + (
                self.odometry.y - 102.73177847690162
            ) ** 2 < (self.odometry.x - 124.6465275631675) ** 2 + (
                self.odometry.y - 239.69487719818875
            ) ** 2:
                # k-city2 @ 작은거
                self.publish_ref_path(
                    wx=[38.96862971336143, 19.71714583610016],
                    wy=[102.73177847690162, 112.70182007206897],
                    num=2,
                )  # 1차선 center line

                self.publish_ref_path(
                    wx=[37.30921904311905, 19.640177483083267],
                    wy=[101.29942025157435, 110.61280422263921],
                    num=1,
                )  # 2차선 center line
                self.check_obstacle("small")

            else:
                # k-city2 @큰거
                self.publish_ref_path(
                    wx=[
                        124.6465275631675,
                        127.31170621333267,
                        130.41270114770884,
                        133.72936396382883,
                    ],
                    wy=[
                        239.69487719818875,
                        246.26584097348763,
                        255.14942999748067,
                        266.65133363821315,
                    ],
                    num=1,
                )  # 1차선 center line

                self.publish_ref_path(
                    wx=[
                        129.67593645777777,
                        132.3760471815653,
                        134.03048074049175,
                        137.01021579556496,
                    ],
                    wy=[
                        244.2264529842743,
                        250.73229360150705,
                        255.6939326996221,
                        266.30647016359876,
                    ],
                    num=2,
                )  # 2차선 center line
                self.check_obstacle("big")

        self.organize_obstacle_lists()
        self.obs_marker()
        self.line_change()
