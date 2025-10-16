#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32, String, Bool
from erp42_msgs.msg import SerialFeedBack, ControlMessage
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from stanley_cone import Stanley
from .mpc_node_cone import MPC
from scipy.ndimage import gaussian_filter1d
# from tf_transformations import *
import numpy as np
import math as m
import threading
import os
import sqlite3
from DB import DB
import time
from scipy.interpolate import CubicSpline, UnivariateSpline
from scipy import interpolate
import shutil
from datetime import datetime


# ✅ yaw를 직접 보간하지 않고, x(t), y(t) 곡선의 접선 방향으로 계산
def compute_yaw_from_spline(cs_x, cs_y, t_values):
    dx = cs_x.derivative()(t_values)
    dy = cs_y.derivative()(t_values)
    return np.arctan2(dy, dx)

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

class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
        # self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 1.0).value
        # self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.05).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )

    def PIDControl(self, speed, desired_value, min, max):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (
            self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt

        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)
        if self.i_err > 5.0:
            self.i_err = 5.0
        if self.i_err < -5.0:
            self.i_err = -5.0

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)

        return int(np.clip(self.speed, min, max))

class PathHandler():
    def __init__(self, node, path_topic, first_lap_done_topic):
        self.node = node
        self.node.create_subscription(Path, path_topic, self.callback_path, qos_profile_system_default)
        self.node.create_subscription(Bool, first_lap_done_topic, self.first_lap_done_path, qos_profile_system_default)
        

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.db_path = os.path.expanduser("/home/acca/db_file/mpc/path_data.db")
        self.init_db()
        self.path = False
        self.one_lap_done = False
        

    def backup_then_reset_db(self):
        """기존 DB가 있으면 타임스탬프 .bak로 백업하고 원본 제거"""
        try:
            db_dir = os.path.dirname(self.db_path)
            os.makedirs(db_dir, exist_ok=True)

            if os.path.isfile(self.db_path):
                ts = datetime.now().strftime("%Y%m%d-%H%M%S")
                backup_path = f"{self.db_path}.{ts}.db"
                # 백업: move = 백업 + 원본 제거
                shutil.copy2(self.db_path, backup_path)  # ✅ move → copy2 로 변경
                self.node.get_logger().warn(f"[init_db] Found existing DB. Backed up to: {backup_path}")
                os.remove(self.db_path)
                self.node.get_logger().info(
                    f"[init_db] Original DB deleted after backup: {self.db_path}")
        except Exception as e:
            self.node.get_logger().error(f"[init_db] DB backup failed: {e}")


    def init_db(self):
        thread_id = threading.get_ident()
        self.node.get_logger().info(f'[init_db] Thread ID : {thread_id}')

        # ✅ 여기가 핵심: 테이블 만들기 전에 백업/초기화
        self.backup_then_reset_db()

        with sqlite3.connect(self.db_path) as conn:
            cur = conn.cursor()
            cur.execute('''
                CREATE TABLE IF NOT EXISTS path (
                    path_id  CHAR(4),
                    idx INTEGER PRIMARY KEY AUTOINCREMENT,
                    x REAL, y REAL, yaw REAL, speed REAL
                )
            ''')

    def resample_path(self, cx, cy, spacing=0.005):
        """균일한 간격(spacing)으로 (x, y) 경로 리샘플링"""
        
        # 누적 거리 계산
        dx = np.diff(cx)
        dy = np.diff(cy)
        dist = np.sqrt(dx**2 + dy**2)
        cumulative = np.insert(np.cumsum(dist), 0, 0)

        total_length = cumulative[-1]
        if total_length < spacing:
            return cx, cy  # 너무 짧은 경로는 리샘플링하지 않음

        n_samples = int(total_length / spacing)
        uniform_dist = np.linspace(0, total_length, n_samples)

        fx = interpolate.interp1d(cumulative, cx, kind='linear')
        fy = interpolate.interp1d(cumulative, cy, kind='linear')

        rx = fx(uniform_dist)
        ry = fy(uniform_dist)

        return rx.tolist(), ry.tolist()

    def first_lap_done_path(self, msg):
        self.one_lap_done = msg.data

        thread_id = threading.get_ident()
        self.node.get_logger().info(f"[done_path] Thread ID: {thread_id}")

        if msg.data:
            try:
                str_time = time.time()               
                 
                with sqlite3.connect(self.db_path) as conn:

                    self.node.get_logger().info(f"[done_path] Opened DB connection in thread ID: {thread_id}")
                    cur = conn.cursor()

                    # ✅ 중복 제거
                    unique_coords = set()
                    filtered_x, filtered_y = [], []

                    for x_val, y_val in zip(self.cx, self.cy):
                        key = (round(x_val, 1), round(y_val, 1))
                        if key not in unique_coords:
                            unique_coords.add(key)
                            filtered_x.append(x_val)
                            filtered_y.append(y_val)

                    raw_cx, raw_cy = filtered_x, filtered_y

                    # ✅ 리샘플링 적용
                    self.cx, self.cy = self.resample_path(raw_cx, raw_cy, spacing=0.1)

                    # 기존 경로
                    x = np.array(self.cx)
                    y = np.array(self.cy)
                    yaw_list = []
                    for row in self.way.poses:
                        q = row.pose.orientation
                        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                        yaw_list.append(yaw)

                    # 기존 경로 저장
                    min_len = min(len(self.cx), len(self.cy), len(self.way.poses))
                    min_len -= 5
                    for i in range(min_len):
                        x = self.cx[i]
                        y = self.cy[i]
                        q = self.way.poses[i].pose.orientation
                        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                        
                        # 기본 속도 설정
                        speed = 12 # 9 modified 25.10.09  

                        cur.execute(
                            "INSERT INTO Path (path_id, x, y, yaw, speed) VALUES (?, ?, ?, ?, ?)",
                            ("A1A2", x, y, yaw, speed)
                        )

                    conn.commit()
                    end_time = time.time()
                    print("latency", end_time-str_time)
                    self.node.get_logger().info("[done_path] Path with curvature-based speed saved to DB.")
                    self.path = True

            except Exception as e:
                self.node.get_logger().warn(f"[done_path] DB error: {e}")

    def callback_path(self, msg):
        self.way = msg
        self.cx, self.cy, self.cyaw = self.update_path(msg)

    def update_path(self, data):
        cx = []
        cy = []
        cyaw = []
        for p in data.poses:
            cx.append(p.pose.position.x)
            cy.append(p.pose.position.y)
            _, _, yaw = euler_from_quaternion([
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w
            ])
            cyaw.append(yaw)
        return cx, cy, cyaw

class State():
    def __init__(self, node, odom_topic):
        node.create_subscription(Odometry, odom_topic, self.callback, qos_profile_system_default)

        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

    def callback(self,msg):
        self.pose = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])    
        self.v = msg.twist.twist.linear.x

class SpeedSupporter():
    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value, he_gain, ce_gain, he_thr, ce_thr):
        hdr = self.func(abs(hdr), -he_gain, he_thr)
        ctr = self.func(abs(ctr), -ce_gain, ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res
                      


class Drive():
    def __init__(self, node, state, path):
        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile_system_default)
        self.one_lap_done_pub = node.create_publisher(Bool, "one_lap_done", qos_profile_system_default)
        node.create_subscription(Float32, "hdr", self.hdr_callback,qos_profile_system_default)
        node.create_subscription(Float32, "ctr", self.ctr_callback, qos_profile_system_default)
        

        self.hdr = 0.0
        self.ctr = 0.0
        
        self.path = path
        self.state = state
 
        self.st = Stanley()
        self.ss = SpeedSupporter()
        self.pid = PID(node)
        self.mpc = None

        self.first_lap = True


    def set_mpc(self, mpc_instance):
        self.mpc = mpc_instance

    def hdr_callback(self, msg):
        self.hdr = msg.data

    def ctr_callback(self, msg):
        self.ctr = msg.data

    def publish_cmd(self):        
        target_idx, error  = self.st.calc_target_index(self.state, self.path.cx, self.path.cy)
        self.decision_first_lap(target_idx)
        
        ####### First_lap Controller ########
        if not self.path.one_lap_done: 
            if self.decision_last_idx(target_idx):
                h_gain_curve = 0.8
                c_gain_curve = 0.5
                target_speed = 1.0

                steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_curve, c_gain_curve, target_idx, error)
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=2, max_value=4, he_gain=50.0, ce_gain=30.0, he_thr=0.001, ce_thr=0.002)
                if self.state.v * 3.6 >= adapted_speed:
                    input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                else:
                    input_brake = 0
                speed = adapted_speed 
                print("close last idx", adapted_speed)

            else:
                if self.decision_straight(target_idx):
                    # h_gain_straight = 0.5
                    # c_gain_straight = 0.24

                    h_gain_straight = 0.6
                    c_gain_straight = 0.3
                    # target_speed = 5.0
                    target_speed = 12.0 # origin (12.0) modified 25.10.04

                    steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_straight, c_gain_straight, target_idx, error)
                    adapted_speed = self.ss.adaptSpeed(
                        target_speed,
                        hdr,
                        ctr,
                        min_value=9,
                        max_value=12,
                        he_gain=40.0,
                        ce_gain=30.0,
                        he_thr=0.07,
                        ce_thr=0.05
                        )
                    # adapted_speed = self.ss.adaptSpeed(
                    #     target_speed,
                    #     hdr,
                    #     ctr,
                    #     min_value=8, 
                    #     max_value=15,
                    #     he_gain=40.0, 
                    #     ce_gain=30.0, 
                    #     he_thr=0.07, 
                    #     ce_thr=0.05
                    #     ) # modified 25.10.04
                    
                    # origin before 1004 
                        # min_value=9,
                        # max_value=12,
                    
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = adapted_speed

                    print("straight", adapted_speed)


                else:
                    h_gain_curve = 0.8
                    c_gain_curve = 0.5
                    target_speed = 6.0 # origin (7.0) modified 25.10.04

                    steer, hdr, ctr = self.st.stanley_control(self.state, self.path.cyaw, h_gain_curve, c_gain_curve, target_idx, error)
                    adapted_speed = self.ss.adaptSpeed(
                        target_speed, 
                        hdr, 
                        ctr, 
                        min_value=5, # origin (6) modified 25.10.04
                        max_value=6,  # origin(7) modified 25.10.04
                        he_gain=50.0, 
                        ce_gain=30.0, 
                        he_thr=0.001, 
                        ce_thr=0.002
                        )
                    if self.state.v * 3.6 >= adapted_speed:
                        input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                    else:
                        input_brake = 0
                    speed = adapted_speed
                    print("curve", adapted_speed)

        ####### Second_lap Controller ########
        else:
            # modified 25.10.08 (직선 판단 불필요)          
            _, steer, speed_output = self.mpc.pose_callback(self.state.pose)
            kspeed = speed_output * 3.6
            
            ## Fallback Logic (Stanley Controlller) ##
            if self.hdr != 0 and self.ctr != 0:
                print("\nStanley_warning\n")
                adapted_speed = self.ss.adaptSpeed(
                    kspeed, 
                    self.hdr, 
                    self.ctr, 
                    min_value=5, 
                    max_value=10, 
                    he_gain=40.0, 
                    ce_gain=30.0, 
                    he_thr=0.07, 
                    ce_thr=0.05
                    )
                if self.state.v * 3.6 >= adapted_speed:
                    input_brake = (abs(self.state.v * 3.6 - adapted_speed) / 20.0) * 200
                else:
                    input_brake = 0
                speed = adapted_speed
            
            ## MPC ##
            else:
                if self.state.v * 3.6 >= kspeed:
                    input_brake = (abs(self.state.v * 3.6 - kspeed) / 20.0) * 200
                else:
                    input_brake = 0
                speed = kspeed

            # speed = self.pid.PIDControl(self.state.v * 3.6, speed, 0, 25)


        msg = ControlMessage()
        msg.speed = int(speed)*10 
        msg.steer = int(m.degrees((-1)*steer) * 1e3)
        msg.gear = 2
        msg.brake = int(input_brake)

        self.pub.publish(msg)

    def decision_first_lap(self, target_idx):
        '''
        True:\n
        생성된 path가 10m 이상이고 target_idx가 10 이하(출발점 부근)일 때
        '''
        if len(self.path.cyaw) >= 100 and target_idx <= 10 and self.first_lap: 
            self.first_lap = False
            self.one_lap_done_pub.publish(Bool(data=True))

    def decision_last_idx(self, target_idx):
        ''' 
        True:\n
        path의 마지막 노드랑 차랑 인덱스가 10개(1m) 이내일때 
        '''
        if  abs(len(self.path.cyaw) - target_idx) <= 10: 
            return True
        else:
            return False
        

    def decision_straight(self, target_idx):
        yaw_list = []
        for i in range(target_idx - 5, target_idx + 30):
            try:
                yaw_list.append(self.path.cyaw[i])
            except IndexError:
                break
        mean = np.mean(np.abs(np.diff(yaw_list)))
        # print(mean)
        if mean > 0.0075: #1027 0.01 -> 0.0075 -> 0.015 -> 0.02 --> 0.0075
            return False
        else:
            return True
        

def main(args = None):
    rclpy.init(args = args)
    
    node = rclpy.create_node("driving_node")
    state = State(node, "/odometry/navsat")
    path_tracking = PathHandler(node, "del_path", "one_lap_done")
    d = Drive(node, state, path_tracking)
    
    thread = threading.Thread(target=rclpy.spin, args= (node, ), daemon = True)
    thread.start()

    rate = node.create_rate(10)  # 10Hz

    mpc_ready = False
    first = True
    db = None
    db_path = "/mpc/path_data.db"
       

    while rclpy.ok():
        try:
            if not mpc_ready and path_tracking.path and first:
                if db is None:  # db가 아직 없다면 생성
                    
                    db = DB(db_path)
                    first = False
                    
            if not mpc_ready and path_tracking.path:
                mpc = MPC(db)
                d.set_mpc(mpc)
                mpc_ready = True 
            d.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()
    
    
    
if __name__=="__main__":
    main()