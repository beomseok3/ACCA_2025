import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from erp42_msgs.msg import SerialFeedBack, ControlMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from DB import DB
import numpy as np
import math

from enum import Enum
import threading

from pure_pursuit import *


class Mission(Enum):
    # kcity 본선 대회용 (final - 1012)

    A1A2 = "driving_a"
    A2A3 = "driving_b"
    A3A4 = "driving_c"
    A4A5 = "driving_d"
    # A4A5 = "driving_d"
    A5A6 = "driving_e"
    A6A7 = "driving_f"
    # A6A7 = "driving_f"
    A7A8 = "driving_g"
    A8A9 = "driving_h"
    A9A10 = "driving_i"
    A10A11 = "driving_j"
    # A10A11 = "driving_j"
    A11A12 = "driving_k"
    A12A13 = "driving_l"
    A13A14 = "driving_m"
    A14A15 = "driving_o"
    A15A16 = "driving_p"
    A16A17 = "driving_q"
    A17A18 = "driving_r"
    # A17A18 = "driving_r"
    A18A19 = "driving_s"
    A19A20 = "driving_t"
    A20A21 = "driving_u"
    A21A22 = "driving_v"
    # A21A22 = "driving_v"
    A22A23 = "driving_w"
    A23A24 = "driving_x"
    # A23A24 = "driving_x"
    A24A25 = "driving_y"
    A25A26 = "driving_z"
    A26A27 = "driving_A"
    A27A28 = "driving_B"
    A28A29 = "driving_C"
    A29A30 = "driving_D"
    A30A31 = "driving_E"


class StateMachine(Node):
    def __init__(self):
        super().__init__("statemachine")

        # subscriber
        self.create_subscription(
            SerialFeedBack, "/erp42_feedback", self.feedback_callback, 10
        )
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )

        # publisher
        self.cmd_pub = self.create_publisher(ControlMessage, "cmd_msg", 10)
        self.path_pub = self.create_publisher(Path, "global_path", 10)

        # timer
        self.create_timer(0.01, self.timer)

        # init param
        # --- path--- #
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.cv = []

        # ---path_index--- #
        self.target_ind = 0
        self.current_ind = 0

        # ---odometry & control--- #
        self.odom = State(is_reverse=False)  # pure pursuit
        self.target_course = None  # pure pursuit

        # ---mission_param--- #
        self.mission = Mission.A1A2

        # ---global_path--- #
        self.db = DB("0827_ssupark_ys.db")
        self.cx, self.cy, self.cyaw, self.cv = self.db.query_from_id(self.mission.name)
        self.target_course = TargetCourse(self.cx, self.cy)
        self.target_ind, _ = self.target_course.search_target_index(self.odom)
        self.publish_path()

        # ---target_speed--- #
        self.target_speed = 10

    # callback group
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.odom.x = position.x
        self.odom.y = position.y

        if self.odom.direction == -1:  # 후진 할때
            yaw = (yaw + math.pi) % (2 * math.pi)  # 180도 회전

        self.odom.yaw = yaw
        self.odom.rear_x = self.odom.x - self.odom.direction * (
            (WB / 2) * math.cos(self.odom.yaw)
        )
        self.odom.rear_y = self.odom.y - self.odom.direction * (
            (WB / 2) * math.sin(self.odom.yaw)
        )

    def feedback_callback(self, msg):
        self.odom.v = abs(msg.speed) * 3.6

    # publisher group
    def publish_path(self):
        path_msg = Path()

        path_msg.header.frame_id = "map"

        for x, y, yaw in zip(self.cx, self.cy, self.cyaw):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = (
                0.0  # Assuming the path is on the ground, adjust if needed
            )
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # timer callback
    def timer(self):
        if "driving" in self.mission.value:

            if self.current_ind >= len(self.cx) - 20:
                missions = list(Mission)
                idx = missions.index(self.mission)
                if idx + 1 < len(missions):
                    self.mission = missions[idx + 1]
                    self.cx, self.cy, self.cyaw, self.cv = self.db.query_from_id(
                        self.mission.name
                    )
                    if len(self.cx) == 0:
                        return
                    self.target_course = TargetCourse(self.cx, self.cy)
                    self.target_ind, _ = self.target_course.search_target_index(
                        self.odom
                    )
                    self.current_ind = self.target_course.old_nearest_point_index
                    self.publish_path()
            else:
                # 속도 제어 (P제어)
                target_speed = self.target_speed
                ai = proportional_control(target=target_speed, current=self.odom.v)
                # 조향 제어 (Pure Pursuit)
                di, self.target_ind = pure_pursuit_steer_control(
                    self.odom, self.target_course, self.target_ind
                )
                self.current_ind = self.target_course.old_nearest_point_index

                # ERP42 제어 메시지 작성
                cmd = ControlMessage()
                cmd.speed = int(target_speed) * 10
                cmd.steer = int(np.clip(-math.degrees(di) * np.pi, -28, 28))
                cmd.gear = 0 if self.odom.direction == -1 else 2
                cmd.brake = 0

                self.cmd_pub.publish(cmd)

                # 로그 출력
                print(
                    f"mission {self.mission.name}, {self.mission.value}  : c_ind {self.current_ind} / p_ind {len(self.cx)}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
