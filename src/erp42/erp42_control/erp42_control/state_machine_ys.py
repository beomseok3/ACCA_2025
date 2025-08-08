#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from erp42_msgs.msg import SerialFeedBack, ControlMessage

from Modifier_param import set_param


from stanley import Stanley
from DB import DB
import numpy as np
import math as m


from enum import Enum
import threading


from controller_obstacle_ys import Obstacle
from controller_uturn import Uturn
from controller_parking_ys import Parking



def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw




class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    def PIDControl(self, speed, desired_value, min, max):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, min, max))
    
class SpeedSupporter():
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.002).value


    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res




class State(Enum):
############### YS 0801 ###########################
    A1A2 = "drinving_A" # old(15) new(20)
    A2A3 = "parking_B" # 사선 주차 old(5)
    A3A4 = "curve_C" # old(8) new(11)
    A4A5 = "driving_D" # old(15) new(20)
    A5A6 = "slow_E" # 방지턱 old(12) new(12)
    A6A7 = "curve_F" # old(8) new(11)
    A7A8 = "driving_G" # old(12) new(20)
    B1B2 = "uturn_H" # old(7) 
    A8A9 = "driving_I" # old(12) new(15)
    A9A10 = "curve_I" # old(12) new(15)
    A10A11 = "obstacle_J" # old(5) new(8)
    A12A13 = "parking_J" # old(5) new(8)
###################  YS ###########################

# 2024 kcity YS 대회용 (final - 1012)
    # A1A2 = "driving_a"  #13
    # A2A3 = "curve_b"  #8
    # A3A4 = "driving_c"  #12
    # B1B2 = "uturn_a"  #7
    # A4A5 = "driving_d"  #12
    # A5A6 = "obstacle_a"  #5
    # A6A7 = "curve_e"  #8




class GetPath():
    def __init__(self, db, init_state):
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.cv = []
        self.db = db

        self.file_open_with_id(init_state.name)


    def file_open_with_id(self, id):
        self.cx, self.cy, self.cyaw, self.cv = self.db.query_from_id(id)



class GetOdometry():
    def __init__(self, node, odom_topic, pcl_topic):
        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

        self.odom_x = 0.
        self.odom_y = 0.
        self.odom_yaw = 0.

        self.pcl_x = 0.
        self.pcl_y = 0.
        self.pcl_yaw = 0.

        self.node = node

        self.covariance_threshold = 0.007

        self.node.create_subscription(Odometry, odom_topic, self.callback_odom, qos_profile=qos_profile_system_default)
        self.node.create_subscription(PoseWithCovarianceStamped, pcl_topic, self.callback_pcl, qos_profile=qos_profile_system_default)
        self.node.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile=qos_profile_system_default)
        self.node.create_subscription(NavSatFix, "ublox_gps_node/fix", self.callback_gps, qos_profile=qos_profile_system_default)

    def callback_odom(self,msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        _,_,self.odom_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

    def callback_pcl(self,msg):
        self.pcl_x = msg.pose.pose.position.x
        self.pcl_y = msg.pose.pose.position.y
        _,_,self.pcl_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        
    def callback_erp(self,msg):
        self.v = msg.speed

    def callback_gps(self,msg):
        if msg.position_covariance[0] < self.covariance_threshold and msg.position_covariance[4] < self.covariance_threshold:
            print("kalman")
            self.x, self.y, self.yaw = self.odom_x, self.odom_y, self.odom_yaw
        else:
            print("ndt")
            self.x, self.y, self.yaw = self.pcl_x, self.pcl_y, self.pcl_yaw


class StateMachine():
    def __init__(self, node, odometry, path, state):
        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile=qos_profile_system_default)
        self.path_pub = node.create_publisher(Path, "global_path", qos_profile=qos_profile_system_default)
        self.node = node
        self.state = state
        self.path = path
        self.odometry = odometry

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.target_idx = 0
        self.mission_finish = False


        self.obstacle = Obstacle(self.node)
        self.uturn = Uturn(self.node)
        self.parking = Parking(self.node)

        self.k = 0





    def update_state_and_path(self):
        print(self.target_idx, len(self.path.cx))
        if self.state.value[:-2] == "driving" or self.state.value[:-2] == "curve":
            if self.target_idx >= len(self.path.cx) - 10: # driving에서 state 전환 조건
                states = list(State)
                current_index = states.index(self.state)
                try:
                    self.state = states[current_index + 1]  # state update (driving -> mission)
                    self.path.file_open_with_id(self.state.name) # path update
                    self.publish_path()  # path publish
                    self.mission_finish = False
                except IndexError:
                    print("index out of range")
        else:
            if self.mission_finish: # mission에서 state 전환 조건
                states = list(State)
                current_index = states.index(self.state)
                try:
                    self.state = states[current_index + 1]  # state update (mission -> driving)
                    self.path.file_open_with_id(self.state.name) # path update
                    self.publish_path() # path publish
                except IndexError:
                    print("index out of range")



    def update_cmd_msg(self):
        print(self.state.value)
        msg = ControlMessage()

        if self.state.value[:-2] == "driving":
            if self.odometry.x != 0.: #10.03 수정
                steer, self.target_idx, hdr, ctr = self.st.stanley_control(self.odometry, self.path.cx, self.path.cy, self.path.cyaw, h_gain=0.5, c_gain=0.24)
                target_speed = self.set_target_speed() # TODO min : 5이하로 줄임 (0801 내가함)
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=5, max_value=15) # 에러(hdr, ctr) 기반 목표 속력 조정
                speed = self.pid.PIDControl(self.odometry.v * 3.6, adapted_speed, min=5, max=15) # speed 조정 (PI control) 
                brake = self.cacluate_brake(adapted_speed) # brake 조정

                # msg.speed = int(adapted_speed) * 10
                msg.speed = int(speed) * 10
                msg.steer = int(m.degrees((-1) * steer))
                msg.gear = 2
                msg.brake = int(brake)
            else:
                pass

        elif self.state.value[:-2] == "curve":
            if self.odometry.x != 0.: #10.03 수정
                steer, self.target_idx, hdr, ctr = self.st.stanley_control(self.odometry, self.path.cx, self.path.cy, self.path.cyaw, h_gain=0.5, c_gain=0.24)
                target_speed = self.set_target_speed()
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=7, max_value=12) # 에러(hdr, ctr) 기반 목표 속력 조정
                speed = self.pid.PIDControl(self.odometry.v * 3.6, adapted_speed,  min=7, max=12) # speed 조정 (PI control) 
                brake = self.cacluate_brake(adapted_speed) # brake 조정

                # msg.speed = int(adapted_speed) * 10
                msg.speed = int(speed) * 10
                msg.steer = int(m.degrees((-1) * steer))
                msg.gear = 2
        elif self.state.value[:-2] == "obstacle":
            if self.k < 1:
                try:
                    set_param("bs_cropbox_filter","detection_area","[0.,20.,-2.5,2.5]")

                except:
                    self.k -=1
                else:
                    self.k +=1
            msg, self.mission_finish = self.obstacle.control_obstacle(self.odometry, self.path)
        elif self.state.value[:-2] == "uturn":
            if self.odometry.x != 0.:
                msg, self.mission_finish = self.uturn.control_uturn(self.odometry)
        elif self.state.value[:-2] == "parking":
            if self.odometry.x != 0.:
                msg, self.mission_finish = self.parking.control_parking(self.odometry)
        else:
            print("error: ", self.state.value)

        return msg



    def set_target_speed(self):
        target_speed = self.path.cv[self.target_idx]
        return target_speed


    def cacluate_brake(self, adapted_speed): # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.odometry.v * 3.6 >= adapted_speed:
            brake = (abs(self.odometry.v * 3.6 - adapted_speed) / 20.0) * 200
            brake = np.clip(brake, 0, 100)
        else:
            brake = 0
        return brake


    def publish_cmd(self):
        self.update_state_and_path()
        msg = self.update_cmd_msg()
        self.pub.publish(msg)
        

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # Assuming frame_id is 'map', adjust as necessary

        for x, y, yaw in zip(self.path.cx, self.path.cy, self.path.cyaw):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # Assuming the path is on the ground, adjust if needed
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)



def main():
    rclpy.init(args=None)
    node = rclpy.create_node("state_machine_node")

    # Declare Params
    # node.declare_parameter("file_name", "1006_1507_acca.db") #kcity
    # node.declare_parameter("file_name", "1003_dolge_test.db") #dolge
    # node.declare_parameter("file_name", "YS_ndt_test.db")
    node.declare_parameter("file_name", "bunsudae_v1.db")
    node.declare_parameter("odom_topic", "/localization/kinematic_state")
    node.declare_parameter("pcl_topic", "/pcl_pose")


    # Get Params
    file_name = node.get_parameter("file_name").get_parameter_value().string_value
    odom_topic = node.get_parameter("odom_topic").get_parameter_value().string_value
    pcl_topic = node.get_parameter("pcl_topic").get_parameter_value().string_value



    #Declare Instance
    db = DB(file_name)
    state = State.A12A13
    path = GetPath(db, state)
    odometry = GetOdometry(node, odom_topic, pcl_topic)
    state_machine = StateMachine(node, odometry, path, state)
    state_machine.publish_path() # A1A2(초기 path) pubF


    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    rate = node.create_rate(10)

    while rclpy.ok():
        try:
            state_machine.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()

if __name__ == "__main__":
    main()