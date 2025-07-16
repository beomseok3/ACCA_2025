import rclpy
import numpy as np
from geometry_msgs.msg import Quaternion
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.distance import cdist
from nav_msgs.msg import Path, OccupancyGrid
from erp42_msgs.msg import ControlMessage
from erp42_msgs.msg import SerialFeedBack
import os
import sys
#TODO . PP로 바꾸기, 나가는 경로 따로 만들기
# 현재 파일의 디렉토리 경로
current_dir = os.path.dirname(os.path.abspath(__file__))

# 프로젝트 루트 경로 추가
project_root = os.path.abspath(os.path.join("/home/acca/acca_ws/src/costmap_has/costmap_has"))
sys.path.append(project_root)

# 상대 경로 import
from PathPlanning.HybridAStar.hybrid_a_star import (
    hybrid_a_star_planning,
    XY_GRID_RESOLUTION,
    YAW_GRID_RESOLUTION
)

from pure_pursuit import PurePursuit


class State:
    def __init__(self):
        self.x = 0.0    # x 좌표
        self.y = 0.0    # y 좌표
        self.yaw = 0.0  # 방향(라디안)
        self.v = 0.0    # 속도

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # 현재 위치, 맵, filtered_cone_poses를 저장할 변수
        self.current_pose = None
        self.filtered_cone_poses = None

        self.start = None
        self.goal = None

        self.count = 0
        self.count_time = 0
        self.estop = 0
        self.target_speed = 5
        self.target_idx=0
        self.passed_parking = False

        self.planned_path_x_list = None
        self.planned_path_y_list = None
        self.planned_path_yaw_list = None
        self.pp = PurePursuit()
        self.state = State()

        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        # Odometry 구독
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            10)

        # Cone Poses 구독
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            'costmap',
            self.costmap_cb,10)

        self.create_timer(0.033, self.controller_parking)

        self.cmd_pub = self.create_publisher(
            ControlMessage, "cmd_msg", 10
        )


        self.ox=[17,40]
        self.oy=[-3,10]
        self.flag_find_route = False
        self.flag_start_point = False
        self.get_logger().info('Path Planner Node has been initialized.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().debug('Received current pose')
        self.state.x = self.current_pose.position.x
        self.state.y = self.current_pose.position.y
        self.state.yaw = self.euler_from_quaternion(self.current_pose.orientation)
        self.state.v = 1
        ######################################localization point#######################################
        if cdist([[self.current_pose.position.x,self.current_pose.position.y]],[[-8, -41]]) <= 2:
        ###############################################################################################
            self.flag_start_point = True

    def costmap_cb(self, msg):
        self.costmap = msg

        obstacle_threshold = 40  # 임계값 설정
        costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        obstacle_indices = np.where(costmap_data > obstacle_threshold)

        # 장애물 위치를 실제 좌표로 변환
        x_coordinates = msg.info.origin.position.x + obstacle_indices[1] * msg.info.resolution
        y_coordinates = msg.info.origin.position.y + obstacle_indices[0] * msg.info.resolution

        # 장애물 위치를 (x, y) 좌표 쌍의 리스트로 저장
        self.obstacles = list(zip(x_coordinates, y_coordinates))

        # ox와 oy 리스트도 업데이트 (기존 기능 유지)
        self.ox = x_coordinates.tolist()
        self.oy = y_coordinates.tolist()




    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q


    def controller_parking(self):

        print(self.count)
        path = None
        if not self.flag_find_route and self.flag_start_point:

            # 장애물 리스트를 NumPy 배열로 변환
            obstacles = np.array(self.obstacles)

            # 차량의 현재 위치 정보 추출
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            current_yaw = self.euler_from_quaternion(self.current_pose.orientation)

            # 필터 박스 범위 (차량 기준 상대 좌표)
            # 여기에 local_x_min, local_x_max, local_y_min, local_y_max는 차량 기준 값 (회전 변환 없이 적용)
            local_x_min, local_x_max = 3.0, 1.0   # 차량 전방이 아니라, 차량 기준 예: 후방 부분: -3m ~ -1m
            local_y_min, local_y_max = 0.0, 3.0      # 차량 우측: 0m ~ 3m

            rotation_matrix = np.array([
                [np.cos(current_yaw),-np.sin(current_yaw)],
                [np.sin(current_yaw),-np.cos(current_yaw)]
            ])

            corners = np.array([[local_x_min,local_y_min],
                               [local_x_min,local_y_max],
                               [local_x_max,local_y_min],
                               [local_x_max,local_y_max]])
            
            rotated_corners = np.dot(corners, rotation_matrix.T)

            rotated_x_min = np.min(rotated_corners[:,0])
            rotated_x_max = np.max(rotated_corners[:,0])
            rotated_y_min = np.min(rotated_corners[:,1])
            rotated_y_max = np.max(rotated_corners[:,1])

            # 절대 좌표계에서 필터 영역을 계산 (현재 차량 포지션 기준으로 오프셋 적용)
            abs_x_min = current_x + rotated_x_min
            abs_x_max = current_x + rotated_x_max
            abs_y_min = current_y + rotated_y_min
            abs_y_max = current_y + rotated_y_max

            # 필터 박스 범위 내 장애물 확인 (회전 변환 없이)
            if np.any(obstacles):
                in_x_range = (obstacles[:,0] >= abs_x_min) & (obstacles[:,0] <= abs_x_max)
                in_y_range = (obstacles[:,1] >= abs_y_min) & (obstacles[:,1] <= abs_y_max)
                in_range = in_x_range & in_y_range

                is_ob = 1 if np.any(in_range) else 0
                self.get_logger().info(f"장애물 필터링: 전체 장애물 {len(obstacles)}개 중, 박스 내 {np.sum(in_range)}개 있음")

                # 주차 위치를 찾았는가?
                if not is_ob:
                    
                    # 주차위치를 찾고 멈췄는가?
                    if self.count == 1:
                        start_yaw = self.euler_from_quaternion(self.current_pose.orientation)
                        ############################## localization point ########################
                        self.start = [
                            self.current_pose.position.x + 7*np.cos(start_yaw),
                            self.current_pose.position.y + 7*np.sin(start_yaw),
                            start_yaw]
                        ############################################################################

                        if len(obstacles) > 0:
                            distances = np.linalg.norm(obstacles - np.array([current_x, current_y]), axis=1)
                            closest_obstacle_idx = np.argmin(distances)
                            closest_obstacle_x, closest_obstacle_y = obstacles[closest_obstacle_idx]
                            print(closest_obstacle_x, closest_obstacle_y)

                        # 주차 위치를 가장 가까운 장애물에서 현재 방향 기준으로 설정
                            parking_distance_x = 2.5  # x축으로 2m 떨어진 위치
                            parking_distance_y = 1.5 # y축으로 1m 떨어진 위치

                            obstacle_direction = math.atan2(closest_obstacle_y - current_y, closest_obstacle_x - current_x)
                        

                        # 목표점 설정 (현재 위치에서 앞으로 2m, 옆으로 3m)
                        ######################### localization point ###############################

                        self.goal = [
                            closest_obstacle_x + parking_distance_x * np.cos(start_yaw) + parking_distance_y * np.sin(start_yaw),
                            closest_obstacle_y + parking_distance_x * np.sin(start_yaw) - parking_distance_y * np.cos(start_yaw),
                            start_yaw 
                        ]
                        
                        #########################localization point###############################

                        # Hybrid A* 경로 계획

                        path = hybrid_a_star_planning(self.start, self.goal, self.ox, self.oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

                        if path:
                            # ROS Path 메시지로 변환
                            self.ros_path = Path()
                            self.ros_path.header.frame_id = 'map'
                            self.ros_path.header.stamp = self.get_clock().now().to_msg()
                            self.planned_path_x_list = path.x_list
                            self.planned_path_y_list = path.y_list
                            self.planned_path_yaw_list = path.yaw_list

                            for x, y, yaw in zip(path.x_list, path.y_list, path.yaw_list):
                                pose = PoseStamped()
                                pose.header = self.ros_path.header
                                pose.pose.position.x = x
                                pose.pose.position.y = y
                                pose.pose.orientation = self.quaternion_from_yaw(yaw)
                                self.ros_path.poses.append(pose)

                            # 경로 발행
                            self.path_publisher.publish(self.ros_path)
                            self.flag_find_route=True
                            self.count += 1
                    elif self.count == 0:
                            msg = ControlMessage(mora=0, estop=1,gear=2,speed = 0*10, steer = 0,brake=200)
                            self.count += 1
                            # return msg, True
                            self.cmd_pub.publish(msg)

                # 주차미션 범위안에 들었는가?
                else:
                    msg = ControlMessage(mora=0, estop=0,gear=2,speed = 5*10, steer = 0,brake=0)
                    self.get_logger().warn('Not yet path found')
                    # return msg,True
                    self.cmd_pub.publish(msg)

            # 아무것도 찾지 못했는가
            else:
                msg = ControlMessage(mora=0, estop=0,gear=2,speed = 5*10, steer = 0,brake=0)
                self.get_logger().debug("go ahead idiot")
                # return msg, True
                self.cmd_pub.publish(msg)

        # 경로를 이미 만들었는가?
        else:
            self.get_logger().debug("it already make paths")
            # 경로가 생성되었는가
            if self.count == 2:
                msg = ControlMessage(mora=0, estop=0,gear=2,speed = 5*10, steer = 0, brake=0)
                self.cmd_pub.publish(msg)
                if cdist([[self.current_pose.position.x,self.current_pose.position.y]],[self.start[:2]]) <= 2:
                    if self.count_time<=20:
                        self.count_time+=1
                    else:
                        self.count_time=0
                        self.count+=1
            
            # 주차의 start위치에 도달했는가?
            elif self.count == 3:
                ####################################### START OF CMD MESSAGE (pure pursuit)#########################################################
                msg = ControlMessage()
                delta, self.target_idx = self.pp.pure_pursuit_control(self.state,
                                                                        self.planned_path_x_list, 
                                                                        self.planned_path_y_list, 
                                                                        reverse=True)
                self.get_logger().debug(f"{self.target_idx}")
                steer = int(math.degrees((-1) * delta))
                

                if self.target_idx >= len(self.planned_path_x_list) - 1: # distance가 0.3m 이내
                    if self.count_time <= 30:
                        self.estop = 0
                        self.count_time += 1
                    else:
                        self.count_time = 0
                        self.estop = 1
                        self.count += 1
                

                msg.steer = int(math.degrees((-1) * steer))
                msg.speed = 30 #int(speed) * 10 
                msg.gear = 0
                msg.estop = self.estop

                # return msg, False
                self.cmd_pub.publish(msg)
                ##################################################### END OF CMD MESSAGE ########################################################

            #주차되었는가?
            else:
                
                # TODO: ESTOP
                if self.flag_find_route:
                    if self.count == 4:
                        if not self.passed_parking:
                            msg = ControlMessage(mora=0, estop=1,gear=2,speed = 0*10, steer = 0, brake=200)
                            self.cmd_pub.publish(msg)
                            self.count_time +=1
                            self.get_logger().warn('Car Parked')
                            if self.count_time == 200:
                                self.passed_parking = True
                                self.count_time = 0
                                self.target_idx = 0
                                self.estop=0
                        else:
                            #############################################################START OF CMD MESSAGE###################################################################
                            msg = ControlMessage()
                            steer, self.target_idx = self.pp.pure_pursuit_control(self.state,
                                                                                    list(reversed(self.planned_path_x_list)), 
                                                                                    list(reversed(self.planned_path_y_list)), 
                                                                                    reverse=False)
                            self.get_logger().debug(f"{self.target_idx}")
                
                            if self.target_idx >= len(self.planned_path_x_list) - 3: # distance가 0.3m 이내
                                if self.count_time <= 30:
                                    self.estop = 0
                                    self.count_time += 1
                                else:
                                    self.count_time = 0
                                    self.estop = 1
                            msg.steer = int(math.degrees( steer))
                            msg.speed = 30 #int(speed) * 10 
                            msg.gear = 2
                            msg.estop = self.estop
                
                            # return msg, False
                            self.cmd_pub.publish(msg)

##

                else:
                    msg = ControlMessage(mora=0, estop=0,gear=2,speed = 5*10, steer = 0, brake=0)
                    self.get_logger().warn("go to parking flag")
                    self.cmd_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
