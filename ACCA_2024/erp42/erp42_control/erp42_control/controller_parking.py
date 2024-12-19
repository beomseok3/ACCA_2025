import sys
import time
from enum import Enum
import numpy as np
import math as m

# ros2
import rclpy
from rclpy.qos import  qos_profile_system_default

#msg
from erp42_msgs.msg import ControlMessage
from geometry_msgs.msg import PoseArray,PoseStamped,Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

#tf
from tf_transformations import *

# stanley
from stanley import Stanley

# DB
try:
    from DB import DB
except Exception as e:
    print(e)


class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_parking", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_parking", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr_parking",0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr_parking",0.002).value

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
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_parking", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_parking", 0.85).value
       
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

class Parking_state(Enum):
    SEARCH = 0
    PARKING = 1
    STOP = 2
    RETURN = 3
    FINISH = 4


class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
    
#sub_cone        
class Detection:
    def __init__(self,node,topic):
        self.node = node
        self.cone = []
        self.node.create_subscription(
            PoseArray,
            topic,
            callback = self.update,
            qos_profile = qos_profile_system_default
        )
    def update(self,msg):
        # print("detection")
        self.cone = msg.poses
    



class Pakring():
    def __init__(self,node):

        self.node = node
        
        # search_path
        # self.search_path_db = DB("1006_search_path_acca.db") #kcity
        # self.search_path_db = DB("1006_search_path_acca.db") #dolge
        self.search_path_db = DB("1012_1344_bs_search_path.db") #dolge

        self.search_path = self.search_path_db.read_db_n("Path", "x", "y", "yaw")
        rows =self.search_path
        self.j =0
        self.search_path = np.array(self.search_path)
        # parking_path
        # self.parking_path_db = DB("1003_1758_parking_path.db") #kcity
        self.parking_path_db = DB("1012_1438_parking_path.db")

        self.parking_path = self.parking_path_db.read_db_n(
            "Path", "x", "y", "yaw"
        )
        #return_path
        # self.return_path_db = DB("1009_2243_return_path.db")
        # self.return_path = self.return_path_db.read_db_n(
        #     "Path","x","y","yaw"
        # )
        # self.return_path = np.array(self.return_path)


        self.parking_path = np.array(self.parking_path)
        
        self.path_cx = [row[0] for row in rows]
        self.path_cy = [row[1] for row in rows]
        self.path_cyaw = [row[2] for row in rows]
        
        # params
        self.standard_point = Pose(
            self.search_path[0][0], self.search_path[0][1], self.search_path[0][2]
        )  
        # kcity : 153 , school : 0922?
        self.min_x = self.standard_point.x +0.0 +13.89150679316159
        self.max_x = self.standard_point.x + 28.0 +13.89150679316159
        self.min_y = self.standard_point.y -2.5 -0.49272527596502247
        self.max_y = self.standard_point.y  -1.0 -0.49272527596502247

        self.marker_id = 1

        self.state = None
        
        # instance
        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)
        self.dt = Detection(node,"/cone_pose_map")
        
        # parking_state_machine
        self.parking_state = Parking_state.SEARCH
        self.goal = 0
        self.target_idx = 0
        self.stop_start_time = 0
        self.low_y_cone = []
        self.search2parking = 0

        # # visualize
        # self.low_y_cone_marker_timer = node.create_timer(1.0, self.marker_timer)
        
        # self.path_timer = noede.create_timer(1,self.path_for_visu)
        
        self.pub_path = node.create_publisher(Path,"parking_path",qos_profile_system_default)
        
        self.pub_low_y_cone_marker = node.create_publisher(MarkerArray, "low_y_cone", qos_profile_system_default)
        
        self.goal_pose_marker = node.create_publisher(Marker,"marker_goal",qos_profile_system_default)
        
        self.standard_point_marker = node.create_publisher(Marker,"standard_point",qos_profile_system_default)
        
        self.detection_area_marker_array = node.create_publisher(MarkerArray,"detection_area",qos_profile_system_default)
        self.marker_dt()

        self.path_for_visu()
    
        
        

    
    def rotate_points(self, points, angle, origin):
        if points is not None:

            angle_radians = -angle  # 시계방향으로 회전하기 위함
            rotation_matrix = np.array(
                [
                    [np.cos(angle_radians), -np.sin(angle_radians)],
                    [np.sin(angle_radians), np.cos(angle_radians)],
                ]
            )

            translated_points = points - origin
            rotated_points = np.dot(translated_points, rotation_matrix.T)
            rotated_points += origin
            return rotated_points

    def euclidean_duplicate(self, p1):

        threshold = 1.0
        for p2 in self.low_y_cone:
            # print(p2,p1)
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False
    
    def in_detection_area(self, point):

        if self.min_x <= point[0] <= self.max_x:
            if self.min_y <= point[1] <= self.max_y:
                return True
        return False
    # visualize
    def marker_dt(self):
        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = self.node.get_clock().now().to_msg()
        
        # marker.ns = "standard_point"
        # marker.id = 100
        # marker.type = Marker.SPHERE
        
        # marker.action = Marker.ADD
        # # Marker의 위치 설정
        # marker.pose.position.x = self.standard_point.x
        # marker.pose.position.y = self.standard_point.y
        # marker.pose.position.z = 1.0
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0
        #  # Marker의 크기 설정 (x, y, z 방향의 크기)
        # marker.scale.x = 0.5
        # marker.scale.y = 0.5
        # marker.scale.z = 0.5

        # # Marker의 색상 설정 (R, G, B, A 값)
        # marker.color.r = 1.0
        # marker.color.g = 0.0
        # marker.color.b = 0.0
        # marker.color.a = 1.0

        # # Marker의 수명을 무한대로 설정
        # marker.lifetime.sec = 0
        # marker.lifetime.nanosec = 0

        # # 퍼블리시
        # self.standard_point_marker.publish(marker)

        combinations = [
    (self.min_x, self.min_y),  # Bottom-left
    (self.max_x, self.min_y),  # Bottom-right
    (self.max_x, self.max_y),  # Top-right
    (self.min_x, self.max_y),  # Top-left
]
        combinations = np.array(combinations)

        combinations = self.rotate_points(combinations,-self.standard_point.yaw,np.array([self.standard_point.x,self.standard_point.y]))

        
        # Create marker array for both spheres and the line strip
        marker_array = MarkerArray()

        # Create a line strip marker for the outline
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.node.get_clock().now().to_msg()
        line_marker.ns = "dt_area_outline"
        line_marker.id = self.marker_id
        self.marker_id += 1  # Increment unique marker ID
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0  # Identity quaternion
        line_marker.scale.x = 0.25  # Thickness of the lines
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0  # Set the line color to red
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0

        # Add the corner points to the line strip
        for x, y in combinations:
            point = Point()
            point.x = x
            point.y = y
            point.z = -1.0
            line_marker.points.append(point)

        # Close the loop by adding the first point again
        first_point = Point()
        first_point.x = combinations[0][0]
        first_point.y = combinations[0][1]
        first_point.z = 0.0
        line_marker.points.append(first_point)

        # Add the line strip to the marker array
        marker_array.markers.append(line_marker)



        # Publish the marker array
        self.detection_area_marker_array.publish(marker_array)
    
    def marker_cone(self):
        if self.low_y_cone:
            marker_array_ = MarkerArray()
            origin_low_y_cone = self.rotate_points(
                np.array(self.low_y_cone),
                -self.standard_point.yaw,
                np.array([self.standard_point.x, self.standard_point.y]),
            )
            for x, y in origin_low_y_cone:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.ns = "cone"
                marker.id = self.marker_id
                self.marker_id += 1  # 고유한 마커 ID 설정
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x  # 각 마커를 서로 다른 위치에 배치
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

                marker_array_.markers.append(marker)
            self.pub_low_y_cone_marker.publish(marker_array_)
    
    def path_for_visu(self):
        path = Path()
        path.header = Header()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y, yaw in zip(self.path_cx,self.path_cy,self.path_cyaw):
                pose = PoseStamped()
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)
        self.pub_path.publish(path)
    
    def detection(self):
        for p1 in [(pose.position.x, pose.position.y) for pose in self.dt.cone]:
            rotated_p1 = self.rotate_points(
                np.array([p1]),
                self.standard_point.yaw,
                np.array([self.standard_point.x, self.standard_point.y]),
            )
            rotated_p1 = tuple(rotated_p1[0])
            if not self.euclidean_duplicate(rotated_p1):
                if self.in_detection_area(rotated_p1):
                    self.low_y_cone.append(rotated_p1)
                    self.marker_cone()

                    self.update_parking_path()


    def update_parking_path(self):
        if self.j ==0:
            for i in range(len(self.low_y_cone) - 1):
                self.low_y_cone = sorted(self.low_y_cone,key= lambda x : x[0])
                dist = self.low_y_cone[i + 1][0] - self.low_y_cone[i][0]
                print(f"{dist} m")
                if dist > 3.1:
                    self.idx = i
                    self.adjust_parking_path()
                    self.j += 1 
                    break

    def adjust_parking_path(self):

        angle =   self.parking_path[-1][2] - self.standard_point.yaw
        
        goal_pose = self.rotate_points(
            np.array(
                [self.low_y_cone[self.idx][0] + 2.0, self.low_y_cone[self.idx][1] - 1.0]
            ),
            -self.standard_point.yaw,
            np.array([self.standard_point.x, self.standard_point.y]),
        )

        dx = goal_pose[0] - self.parking_path[0][0]
        dy = goal_pose[1] - self.parking_path[0][1]

        self.parking_path[:, 0] += dx
        self.parking_path[:, 1] += dy
        
        self.parking_path[:, :2] = self.rotate_points(
            self.parking_path[:, :2], angle, self.parking_path[0][:2]
        )  
        
        a = self.search_path_db.find_idx(
            self.parking_path[-1][0], self.parking_path[-1][1], "Path"
        )
        self.goal = len(self.search_path) - a - 1
        self.node.get_logger().info(f"{self.goal} is made")
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        
        marker.action = Marker.ADD
        # Marker의 위치 설정
        marker.pose.position.x = self.search_path[a][0]
        marker.pose.position.y = self.search_path[a][1]
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
         # Marker의 크기 설정 (x, y, z 방향의 크기)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Marker의 색상 설정 (R, G, B, A 값)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Marker의 수명을 무한대로 설정
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # 퍼블리시
        self.goal_pose_marker.publish(marker)

    def cacluate_brake(
        self, adapted_speed
    ):  # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.state.v * 3.6 >= adapted_speed:
            brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
        else:
            brake = 0
        return brake
        
    def control_parking(self,State):
        self.state = State

        self.node.get_logger().info(f"{self.parking_state},{self.target_idx},{len(self.path_cx) - 1}")
        if self.parking_state == Parking_state.SEARCH:
            self.detection()
            
        if self.target_idx >= len(self.path_cx) - self.goal - 1:
            

            if self.parking_state == Parking_state.SEARCH:

                self.parking_state = Parking_state(self.parking_state.value + 1)  # SEARCH -> PARKING
                self.goal = 0
                self.path_cx = self.parking_path[::-1, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[::-1, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[::-1, 2]  # 세 번째 열 (cyaw 값들)
                self.path_for_visu()
                self.target_idx = 0
                self.search2parking = time.time()
                
                msg = ControlMessage(mora=0, estop=1,gear=0,speed = 0*10, steer = 0,brake=200)
                return msg, False

            elif self.parking_state == Parking_state.PARKING:
                self.parking_state = Parking_state(self.parking_state.value + 1)  # PARKING -> STOP #TODO:return_path는 안에서 밖으로 가는걸로 다시만들기(조향이 달라서 그런지 잘 못 따라감
                self.stop_start_time = time.time()  # STOP 상태로 전환된 시간을 기록
                self.target_idx = 0
                msg = ControlMessage(mora=0, estop=1,gear=2,speed = 0*10, steer = 0,brake=200)
                return msg, False
            
            elif self.parking_state == Parking_state.RETURN:
                msg = ControlMessage(mora=0, estop=1,gear=2,speed = 0*10, steer = 0,brake=200)
                return msg,True
            

        
        else:
            if self.parking_state == Parking_state.STOP:

                if time.time() - self.stop_start_time >= 5.0:
                    self.parking_state = Parking_state(self.parking_state.value + 1)
                    self.target_idx = 0  # STOP -> RETURN
                
                self.search_path = np.array(self.search_path)
                self.path_cx = self.parking_path[::1, 0]  # 첫 번째 열 (cx 값들)
                self.path_cy = self.parking_path[::1, 1]  # 두 번째 열 (cy 값들)
                self.path_cyaw = self.parking_path[::1, 2]  # 세 번째 열 (cyaw 값들)
                self.path_for_visu()
                msg = ControlMessage(mora=0, estop=1,gear=0,speed = 0*10, steer = 0,brake=200)

                return msg, False

            # SEARCH, PARKING, RETURN
            elif self.parking_state == Parking_state.PARKING:                
                if time.time() - self.search2parking <= 5.0:
                    msg = ControlMessage(mora=0, estop=1,gear=0,speed = 0*10, steer = 0,brake=200)
                    return msg, False                 
                    
                try:
                    steer, self.target_idx, hdr,ctr = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        h_gain = 1.5,
                        c_gain = 1.2,
                        reverse=True,
                    )
                    target_speed = 2.0
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=2, max_value=3
                    )
                    brake = self.cacluate_brake(adapted_speed)
                    msg = ControlMessage(mora=0, estop=0,gear=0,speed = int(adapted_speed)*10, steer = int(m.degrees(-1* steer)),brake= int(brake))
                except Exception as e:
                    print(f"Stanley:{e}\n",f"{State}")
                return msg, False
                
            elif self.parking_state == Parking_state.SEARCH:    
                try:
                    steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        h_gain=0.5,
                        c_gain =0.24,
                        reverse=False,
                    )


                    target_speed = 5.0
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=4, max_value=6)
                    speed = self.pid.PIDControl(State.v * 3.6, adapted_speed)
                    msg = ControlMessage(mora=0, estop=0,gear=2,speed = speed*10, steer = int(m.degrees(-1* steer)),brake=0)

                except Exception as e:
                    print(f"{e}: stanley")
                return msg, False
            
            else:
                try:
                    steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                        State,
                        self.path_cx,
                        self.path_cy,
                        self.path_cyaw,
                        h_gain=2.5,
                        c_gain =2.2,
                        reverse=False,
                    )


                    target_speed = 7.0
                    adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=6, max_value=8
                    )
                    speed = self.pid.PIDControl(State.v * 3.6, adapted_speed)
                    msg = ControlMessage(mora=0, estop=0,gear=2,speed = speed*10, steer = int(m.degrees(-1* steer)),brake=0)

                except Exception as e:
                    print(f"{e}: stanley")
                return msg, False

