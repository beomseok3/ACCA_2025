import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from erp42_msgs.msg import ControlMessage, SerialFeedBack  # SerialFeedBack 추가
from darknet_ros_msgs.msg import BoundingBoxes
from tf_transformations import euler_from_quaternion
import math
import numpy as np
from stanley import Stanley
from visualization_msgs.msg import Marker

class Trafficlight:
    def __init__(self, node):
        self.node = node
        self.st = Stanley()  # Stanley 객체 생성

        # 경로 관련 리스트 초기화
        self.gx_list = []
        self.gy_list = []
        self.gyaw_list = []

        # 주행 상태 관리 변수
        self.state = "driving_A"
        self.target_idx = 0

        # 신호등 관련 상태 변수 초기화
        self.current_signal = None
        self.signal_count = 0
        self.target_signal_count = 3  # 신호 확정에 필요한 최소 반복 횟수
        self.red_light_detected = False
        self.green_light_detected = False 
        self.mission_finish = False
        self.current_odometry = None

        
        # 경로 인덱스 관리
        self.current_index = 0 

        # 퍼블리셔 설정
        self.control_pub = self.node.create_publisher(ControlMessage, "cmd_msg", 10)

        # 경로 구독
        self.path_sub = self.node.create_subscription(Path, "global_path", self.path_callback, 10)

        # 신호등 인식 데이터 구독
        self.traffic_light_sub = self.node.create_subscription(
            BoundingBoxes, "bounding_boxes", self.traffic_light_callback, 10
        )

        
        # erp42_feedback 토픽에서 속도 데이터 구독
        self.feedback_sub = self.node.create_subscription(
            SerialFeedBack, "/erp42_feedback", self.feedback_callback, 10
        )
        
        # feedback 속도 초기화
        self.vehicle_speed = 0

        # traffic_light_sections에서 좌표 부분 제거
        self.traffic_light_sections = [
            # 각 구간에 대한 빨간불 ID, 초록불 ID 설정
            ({"B1", "B3"}, {"B2"}),  # 돌계 테스트용
            
            ({"1401", "1402"}, {"1405"}),  # 첫 번째 신호등
            ({"1401", "1402"}, {"1405"}),  # 두 번째 신호등
            ({"1401"}, {"1400", "1405"}),  # 세 번째 신호등, 초록불 ID로 처리
            ({"1400", "1401", "1402", "1404"}, {"1403"}),  # 네 번째 신호등, 빨간불 ID로 처리
            ({"1301", "1304"}, {"1303"}),  # 다섯 번째 신호등
            ({"1401", "1402"}, {"1405"}),  # 여섯 번째 신호등
            ({"1401", "1402"}, {"1405"})   # 일곱 번째 신호등
        ]
        # 초기화
        self.odometry = None

    def feedback_callback(self, msg):
        # 피드백에서 속도 데이터를 업데이트
        try:
            self.vehicle_speed = msg.speed * 10  # 속도 단위 확인 및 변환 필요
        except Exception as e:
            self.node.get_logger().error(f"피드백 속도 업데이트 실패: {str(e)}")
            self.vehicle_speed = 0  # 오류 발생 시 기본값 설정

    def get_vehicle_speed_from_feedback(self):
        # 저장된 차량 속도를 반환
        if self.vehicle_speed is None or self.vehicle_speed == 0:
            self.node.get_logger().warn("유효한 차량 속도가 없습니다. 기본값 0을 반환합니다.")
            return 0  # 기본값 반환
        return self.vehicle_speed

    # 신호등 ID만 사용
    def get_current_traffic_light_ids(self):
        # 현재 구간의 빨간불 및 초록불 ID 반환
        current_section_idx = self.current_index
        red_ids = self.traffic_light_sections[current_section_idx][0]
        green_ids = self.traffic_light_sections[current_section_idx][1]
        return red_ids, green_ids

    def get_odometry_data(self):
        return self.current_odometry  # 현재 odometry 데이터 반환

        
    def path_callback(self, msg):
        self.gx_list = []
        self.gy_list = []
        self.gyaw_list = []

        for p in msg.poses:
            self.gx_list.append(p.pose.position.x)
            self.gy_list.append(p.pose.position.y)
            _, _, yaw = euler_from_quaternion([
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w
            ])
            self.gyaw_list.append(yaw)

        self.target_idx = 0  # 경로 갱신 시 target_idx 초기화
        
        if not hasattr(self, 'path_logged'):  # 처음에만 출력
            self.node.get_logger().info(f"{len(self.gx_list)}개의 경로 포인트 수신")
            self.path_logged = True  # 출력이 끝난 후 다시 출력하지 않음


    def traffic_light_callback(self, msg):
        detected_signal = None
        confidence = 0.0

        # 이전 상태 저장
        prev_red_light_detected = self.red_light_detected
        prev_green_light_detected = self.green_light_detected

        for box in msg.bounding_boxes:
            detected_signal = box.class_id
            confidence = box.probability
            
            # 신뢰도가 90 이상일 때만 신호 확정
            if confidence >= 0.90:
                red_ids, green_ids = self.get_current_traffic_light_ids()

                # 빨간불 감지 (해당 구간의 빨간불 ID 중 하나일 경우)
                if detected_signal in red_ids:
                    self.red_light_detected = True
                    self.green_light_detected = False

                    # 빨간불 상태가 변경되었을 때만 로그 출력
                    if prev_red_light_detected != self.red_light_detected:
                        self.node.get_logger().info(f"빨간불 확정: {detected_signal}")
                
                # 초록불 감지 (해당 구간의 초록불 ID 중 하나일 경우)
                elif detected_signal in green_ids:
                    self.green_light_detected = True
                    self.red_light_detected = False

                    # 초록불 상태가 변경되었을 때만 로그 출력
                    if prev_green_light_detected != self.green_light_detected:
                        self.node.get_logger().info(f"초록불 확정: {detected_signal}")


        # odometry 정보 가져오기
        odometry = self.get_odometry_data()
        self.control_traffic_light(odometry)


    def control_traffic_light(self, odometry):
        msg = ControlMessage()
        mission_finish = False

        # # 현재 주행 상태가 driving이면 신호등 로직 무시
        # if self.state.value[:-2] == "driving":
        #     # 신호등 감지 로직을 무시하고 주행 지속
        #     self.node.get_logger().info(f"현재 주행 상태: {self.state.value}, 신호등 감지 무시")
        #     msg.speed = int(self.vehicle_speed)  # 주행 속도 유지
        #     msg.estop = 0  # 정지 해제
        #     return msg, mission_finish  # 신호등 로직을 건너뛰고 주행만 처리

        # 미션 완료된 상태에서 신호 상태를 초기화 (다음 신호등 구간에 진입)
        if self.mission_finish:
            self.reset_traffic_light()  # 신호 상태 초기화는 미션 완료 후에만 발생
            self.node.get_logger().info("미션 완료: 신호등 로직 중지, 다음 신호등 구간 준비 완료")
            return msg, True

        # Odometry 데이터가 유효한지 확인
        if odometry is not None:
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry, self.gx_list, self.gy_list, self.gyaw_list, h_gain=0.5, c_gain=0.24
            )

        # 정지선까지의 거리가 달라졌을 때만 출력
        current_distance_to_stop_line = len(self.gx_list) - self.target_idx

        # last_distance_to_stop_line이 없으면 0으로 설정
        self.last_distance_to_stop_line = getattr(self, 'last_distance_to_stop_line', current_distance_to_stop_line)

        if current_distance_to_stop_line != self.last_distance_to_stop_line:
            # self.node.get_logger().info(f"현재 target_idx: {self.target_idx}, gx_list 길이: {len(self.gx_list)}, 정지선까지의 거리: {current_distance_to_stop_line}")
            self.last_distance_to_stop_line = current_distance_to_stop_line
            
            
        # 1. 빨간불 감지 시 처리 (정지 후 초록불 전환 시 처리)
        if self.red_light_detected:
            if current_distance_to_stop_line <= 10:
                msg.speed = 0  # 속도 0으로 설정하여 정지
                msg.estop = 1  # 비상 정지 설정
                self.node.get_logger().info(f"빨간불 확인: 차량 정지, 현재속도 = {msg.speed}")

                # 정지 상태에서만 초록불 확인 후 주행 재개
                if self.green_light_detected:  
                    msg.estop = 0  # 비상 정지 해제
                    msg.speed = 10  # 초록불로 전환되었으므로 차량 다시 주행, 속도 10으로 설정
                    self.node.get_logger().info(f"초록불로 전환: 차량 주행 재개, 속도 = {msg.speed}")
                    
                    # 초록불 전환 후 일정 거리 이상 주행 시 미션 종료 처리
                    if len(self.gx_list) - self.target_idx > 10:
                        mission_finish = True
                        self.node.get_logger().info("초록불 전환 후 일정 거리 주행 완료, 미션 종료")
            else:
                self.speed = (current_distance_to_stop_line / len(self.gx_list)) * self.vehicle_speed
                self.speed = int(np.clip(self.speed, 1, int(self.vehicle_speed)))
                msg.speed = self.speed
                msg.estop = 0  # 비상 정지 해제
                self.node.get_logger().info(f"빨간불 감지: 차량 감속, 현재속도 = {msg.speed}")

        # 2. 초록불만 감지된 경우 처리 (정지할 필요 없이 계속 주행)
        elif self.green_light_detected and not self.red_light_detected:
            msg.speed = 12
            # msg.speed = int(self.vehicle_speed)  # 현재 속도 유지
            msg.estop = 0  # 정지 해제
            self.node.get_logger().info(f"초록불 감지: 주행 지속, 속도 유지 = {msg.speed}")

            # 일정 거리 후 미션 종료 조건 추가
            distance_to_finish = len(self.gx_list) - self.target_idx
            if distance_to_finish < 10:  # 정지선 근처에서 미션 완료 처리
                mission_finish = True
                self.node.get_logger().info("초록불 주행 중 미션 완료")

        msg.gear = 2  # 기어 설정


        return msg, mission_finish






    def reset_traffic_light(self):
        # 신호 관련 상태를 초기화하는 함수
        self.red_light_detected = False
        self.green_light_detected = False
        self.signal_count = 0  # 신호 카운트 리셋
        self.current_signal = None  # 현재 신호 리셋
        self.node.get_logger().info("신호 상태 초기화 완료")