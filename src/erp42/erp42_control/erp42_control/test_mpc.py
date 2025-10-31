#!/usr/bin/env python3
import math
import numpy as np
from dataclasses import dataclass, field
import cvxpy
# from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from scipy.spatial import transform
import uuid
from enum import Enum
from DB import DB
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from erp42_msgs.msg import ControlMessage
from visualization_msgs.msg import Marker, MarkerArray
from stanley import Stanley
from std_msgs.msg import Float32, String, Int32

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

class State_mpc(Enum):
    A1A2 = "A1A2"
    A2A3 = "A2A3"
    A3A4 = "A3A4"
    A4A5 = "A4A5"
    A5A6 = "A5A6"
    A6A7 = "A6A7"
    A7A8 = "A7A8"
    A8A9 = "A8A9"
    A9A10 = "A9A10"
    A10A11 = "A10A11"
    A11A12 = "A11A12"
    A12A13 = "A12A13"
    A13A14 = "A13A14"
    A14A15 = "A14A15"
    A15A16 = "A15A16"
    A16A17 = "A16A17"
    A17A18 = "A17A18"
    A18A19 = "A18A19"
    A19A20 = "A19A20"
    A20A21 = "A20A21"
    A21A22 = "A21A22"
    A22A23 = "A22A23"
    A23A24 = "A23A24"
    A24A25 = "A24A25"
    A25A26 = "A25A26"
    A26A27 = "A26A27"
    A27A28 = "A27A28"
    A28A29 = "A28A29"
    A29A30 = "A29A30"
    A30A31 = "A30A31"
    A31A32 = "A31A32"
    A32A33 = "A32A33"
    A33A34 = "A33A34"
    A34A35 = "A34A35"
    A35A36 = "A35A36"
    A36A37 = "A36A37"
    A37A38 = "A37A38"
    A38A39 = "A38A39"
    A39A40 = "A39A40"


@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = [steering speed, acceleration]
    TK: int = 25 # finite time horizon length - kinematic

    # Rk: list = field(
    #     default_factory=lambda: np.diag([0.1, 100.0])
    # ) 
    # # input difference cost matrix, penalty for change of inputs - [accel, steering_speed]
    # Rdk: list = field(
    #     default_factory=lambda: np.diag([0.1, 100.0])
    # )  

    # # (x, y, v, yaw)
    # Qk: list = field(
    #     default_factory=lambda: np.diag([13.0, 13.0, 5.5, 13.0])
    # )
    # # final state error matrix, penalty  for the final state constraints: (x, y, v, yaw)
    # Qfk: list = field(
    #     default_factory=lambda: np.diag([13.0, 13.0, 5.5, 13.0])
    # )

    #     # TODO: you may need to tune the following matrices
    # Rk: list = field(
    #     # default_factory=lambda: np.diag([0.01, 100.0])
    #     default_factory=lambda: np.diag([0.0144, 248.526])
    # )  # input cost matrix, penalty for inputs - [accel, steering_speed]
    # Rdk: list = field(
    #     # default_factory=lambda: np.diag([0.01, 100.0])
    #     default_factory=lambda: np.diag([0.0169, 200]) # (0.0169, 325)
    # )  # input difference cost matrix, penalty for change of inputs - [accel, steering_speed]
    # Qk: list = field(
    #     default_factory=lambda: np.diag([15.4214, 16.0728, 22, 14.8503])  # levine sim
    #     # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
    # )  # state error cost matrix, for the the next (T) prediction time steps [x, y, delta, v, yaw, yaw-rate, beta]
    # Qfk: list = field(
    #     default_factory=lambda: np.diag([19.44, 19.44, 6.655, 15.73])  # levine sim
    #     # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
    # )  # final state error matrix, penalty  for the final state constraints: [x, y, delta, v, yaw, yaw-rate, beta]
    # # # ---------------------------------------------------
        # TODO: you may need to tune the following matrices
    Rk: list = field(
        # default_factory=lambda: np.diag([0.01, 100.0])
        default_factory=lambda: np.diag([0.0429982, 3589.3])
    )  # input cost matrix, penalty for inputs - [accel, steering_speed]
    Rdk: list = field(
        # default_factory=lambda: np.diag([0.01, 100.0])
        default_factory=lambda: np.diag([0.0815731, 11020])
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering_speed]
    Qk: list = field(
        default_factory=lambda: np.diag([10.5392, 36.9121, 1408, 28.1982])  # levine sim
        # default_factory=lambda: np.diag([50., 50., 5.5, 13.0])
    )  # state error cost matrix, for the the next (T) prediction time steps [x, y, delta, v, yaw, yaw-rate, beta]
    Qfk: list = field(
        default_factory=lambda: np.diag(
            [58.0476, 58.0476, 11.7897, 27.8666]
        )  # levine sim
    )
    DTK: float = 0.1  # time step [s] kinematic
    dlk: float = 0.1
    WIDTH: float = 1.160  # Width of the vehicle [m]
    WB: float = 1.040  # Wheelbase [m]
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad] # expand
    MAX_DSTEER = np.deg2rad(5)  # 1.05 rad/s
    MAX_SPEED: float = float(25/3.6)  # maximum speed [m/s] ~ 5.0 for levine sim
    MIN_SPEED: float = -2.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 100.0  # maximum acceleration [m/ss]


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    delta: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    yawrate: float = 0.0
    beta: float = 0.0


class MPC(Node):
    """
    Implement Kinematic MPC on the car
    """

    def __init__(self, db):
        super().__init__(f"mpc_node_{uuid.uuid4().int % 100000}")

        self.db = db
        self.state = State_mpc.A1A2  # initial state of FSM
        self.config = mpc_config()
        self.st = Stanley()
        self.reset_ws = False  # reset warm start option if state changes
        self.total_global_path = self.file_open_with_id(self.state.name)
        self.total_global_path = np.array(self.total_global_path)
        self.total_global_path[3, :] = self.total_global_path[3, :] / 3.6  # kph → m/s 0609 modified
        self.odelta_v = None
        self.odelta = None
        self.oa = None
        self.waypoints = None
        self.latest_state = self.state  # latest state of FSM
        self.speed_output = 0


        vis_ref_traj_topic = "/ref_traj_marker"
        vis_waypoints_topic = "/waypoints_marker"
        vis_pred_path_topic = "/pred_path_marker"
        # drive_topic = "/cmd_msg"
        # pose_topic = "/localization/kinematic_state"

        # self.pose_sub = self.create_subscription(
        #     Odometry, pose_topic, self.pose_callback, 1
        # )
        # self.pose_sub 
        
        # self.drive_pub = self.create_publisher(ControlMessage, drive_topic, 1)
        # self.drive_msg = ControlMessage()
        self.vis_waypoints_pub = self.create_publisher(Marker, vis_waypoints_topic, 1)
        self.vis_waypoints_msg = Marker()
        self.vis_ref_traj_pub = self.create_publisher(PoseArray, vis_ref_traj_topic, 1)
        self.vis_ref_traj_msg = PoseArray()
        self.vis_pred_path_pub = self.create_publisher(PoseArray, vis_pred_path_topic, 1)
        self.vis_pred_path_msg = PoseArray()
        self.pub_hdr = self.create_publisher(Float32, "hdr", 1)
        self.pub_ctr = self.create_publisher(Float32, "ctr", 1)
        self.pub_error = self.create_publisher(Int32, "/mpc/error", 1)
        self.last_pred_path = None
        self.db_idx = 0
        self.mpc_prob_init()


    def file_open_with_id(self, id):
        return self.db.query_from_id_mpc(id)

    def pose_callback(self, pose_msg):

        
            
        self.vehicle_state = self.update_vehicle_state(pose_msg)
        # 안전 슬라이싱
        start = int(max(0, self.db_idx -100))
        end   = int(min(len(self.total_global_path[0,:]), self.db_idx + 400))
        # self.get_logger().info(f"{start}, {end}")
        # if end - start < 2:
        #     # 최소 2포인트 확보: 좌우로 더 늘려보기
        #     start = max(0, min(start, self.total_global_path.shape[1] - 2))
        #     end   = min(self.total_global_path.shape[1], start + 2)

        self.waypoints = self.total_global_path[:, start:end]
        self.visualize_waypoints_in_rviz()
        self.ref_path, self.target_idx, self.db_idx = self.calc_ref_trajectory(
            self.vehicle_state,
            self.waypoints[0, :],
            self.waypoints[1, :],
            self.waypoints[2, :],
            self.waypoints[3, :],
            self.waypoints[4, :]
            )
        # print(self.db_idx)
        # print(f"mpc/-------------waypoints : {len(self.target_idx[0, : ])}------------")

        self.visualize_ref_traj_in_rviz(self.ref_path)

        

        x0 = [
            self.vehicle_state.x,
            self.vehicle_state.y,
            self.vehicle_state.v,
            self.vehicle_state.yaw,
        ]

        # solve the MPC control problem
        ########################################## 연산 오래걸림 ##########################################
        result = self.linear_mpc_control(self.ref_path, x0, self.oa, self.odelta_v)
        ########################################## 연산 오래걸림 ##########################################

        if result[0] is None or (len(self.total_global_path[0 , :]) - self.db_idx <= 60):
            print("--------------------------stanly-----------------------------------")

            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                self.vehicle_state,
                self.waypoints[0, :],
                self.waypoints[1, :],
                self.waypoints[2, :],
                h_gain=0.5,
                c_gain=0.24,
            )
            self.pub_hdr.publish(Float32(data=hdr))
            self.pub_ctr.publish(Float32(data=ctr))            
            target_speed = self.waypoints[
                3, self.target_idx
            ]
            return steer, target_speed
            
        else:
            (
                self.oa,
                self.odelta_v,
                ox,
                oy,
                oyaw,
                ov,
                state_predict,
            ) = result
            # print(self.oa[0])

            steer_output = self.odelta_v[0]
            self.speed_output = self.vehicle_state.v + self.oa[0] * self.config.DTK

            # print(speed_output)
            # self.get_logger().info(f"{self.vehicle_state.v}")
            # self.drive_msg.gear = 2
            # self.drive_msg.steer = int(steer_output * 180.0 / math.pi * -1)
            # # self.drive_msg.speed = int(1.0 * speed_output * 3.6) * 10
            # speed_val = int(max(0, min(65535, speed_output * 3.6 * 10)))
            # self.drive_msg.speed = speed_val
            # self.drive_pub.publish(self.drive_msg)

            self.pub_hdr.publish(Float32(data=0.0))
            self.pub_ctr.publish(Float32(data=0.0))

            return steer_output, self.speed_output


        


    def update_vehicle_state(self, pose_msg):
        """
        Update the vehicle state from Localization.
        """
        vehicle_state = State()
        vehicle_state.x = pose_msg.pose.pose.position.x
        vehicle_state.y = pose_msg.pose.pose.position.y
        vehicle_state.v = pose_msg.twist.twist.linear.x # 0812 수정
        # vehicle_state.v = self.speed_output  # convert to m/s
        # print(f"{pose_msg.twist.twist.linear.x}, {vehicle_state.v}")

        curr_orien = pose_msg.pose.pose.orientation
        q = [curr_orien.x, curr_orien.y, curr_orien.z, curr_orien.w]
        
        vehicle_state.yaw = math.atan2(
            2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)
        )

        return vehicle_state

    # mpc functions
    def mpc_prob_init(self):
        """
        Create MPC quadratic optimization problem using cvxpy, solver: OSQP
        Will be solved every iteration for control.
        More MPC problem information here: https://osqp.org/docs/examples/mpc.html
        More QP example in CVXPY here: https://www.cvxpy.org/examples/basic/quadratic_program.html
        """
        # Initialize and create vectors for the optimization problem
        # Vehicle State Vector
        self.xk = cvxpy.Variable((self.config.NXK, self.config.TK + 1))  # 4 x 9
        # Control Input vector
        self.uk = cvxpy.Variable((self.config.NU, self.config.TK))  # 2 x 8
        objective = 0.0  # Objective value of the optimization problem
        constraints = []  # Create constraints array

        # Initialize reference vectors
        self.x0k = cvxpy.Parameter((self.config.NXK,))  # 4
        self.x0k.value = np.zeros((self.config.NXK,))

        # Initialize reference trajectory parameter
        self.ref_traj_k = cvxpy.Parameter(
            (self.config.NXK, self.config.TK + 1)
        )  # 4 x 9
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        # Initializes block diagonal form of R = [R, R, ..., R] (NU*T, NU*T)
        R_block = block_diag(
            tuple([self.config.Rk] * self.config.TK)
        )  # (2 * 8) x (2 * 8)

        # Initializes block diagonal form of Rd = [Rd, ..., Rd] (NU*(T-1), NU*(T-1))
        Rd_block = block_diag(
            tuple([self.config.Rdk] * (self.config.TK - 1))
        )  # (2 * 7) x (2 * 7)

        # Initializes block diagonal form of Q = [Q, Q, ..., Qf] (NX*T, NX*T)
        Q_block = [self.config.Qk] * (self.config.TK)  # (4 * 8) x (4 * 8)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))  # (4 * 9) x (4 * 9), Qk + Qfk

        # Formulate and create the finite-horizon optimal control problem (objective function)
        # The FTOCP has the horizon of T timesteps

        # --------------------------------------------------------
        # TODO: fill in the objectives here, you should be using cvxpy.quad_form() somehwhere

        # Objective part 1: Influence of the control inputs: Inputs u multiplied by the penalty R
        objective += cvxpy.quad_form(
            cvxpy.vec(self.uk), R_block
        )  # # cvxpy.vec() - Flattens the matrix X into a vector in column-major order

        # Objective part 2: Deviation of the vehicle from the reference trajectory weighted by Q, including final Timestep T weighted by Qf
        objective += cvxpy.quad_form(cvxpy.vec(self.xk - self.ref_traj_k), Q_block)

        # Objective part 3: Difference from one control input to the next control input weighted by Rd
        objective += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.uk, axis=1)), Rd_block)

        # --------------------------------------------------------

        # Constraints 1: Calculate the future vehicle behavior/states based on the vehicle dynamics model matrices
        # Evaluate vehicle Dynamics for next T timesteps
        A_block = []
        B_block = []
        C_block = []
        # init path to zeros
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))  # 4 x 9
        for t in range(self.config.TK):  # 8
            A, B, C = self.get_model_matrix(
                path_predict[2, t],
                path_predict[3, t],
                0.0,  # reference steering angle is zero
            )
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))  # 32 x 32
        B_block = block_diag(tuple(B_block))  # 32 x 16
        C_block = np.array(C_block)  # 32 x 1
        # creating the format of matrices

        # [AA] Sparse matrix to CVX parameter for proper stuffing
        # Reference: https://github.com/cvxpy/cvxpy/issues/1159#issuecomment-718925710
        m, n = A_block.shape  # 32, 32
        self.Annz_k = cvxpy.Parameter(
            A_block.nnz
        )  # nnz: number of nonzero elements, nnz = 128
        data = np.ones(self.Annz_k.size)  # 128 x 1, size = 128, all elements are 1
        rows = A_block.row * n + A_block.col  # No. ? element in 32 x 32 matrix
        cols = np.arange(
            self.Annz_k.size
        )  # 128 elements that need to be care - diagonal & nonzero, 4 x 4 x 8
        Indexer = csc_matrix(
            (data, (rows, cols)), shape=(m * n, self.Annz_k.size)
        )  # (rows, cols)	data

        # Setting sparse matrix data
        self.Annz_k.value = A_block.data

        # Now we use this sparse version instead of the old A_block matrix
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")
        # https://www.cvxpy.org/api_reference/cvxpy.atoms.affine.html#cvxpy.reshape

        # Same as A
        m, n = B_block.shape  # 32, 16 = 4 x 8, 2 x 8
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)  # nnz = 64
        data = np.ones(self.Bnnz_k.size)  # 64 = (4 x 2) x 8
        rows = B_block.row * n + B_block.col  # No. ? element in 32 x 16 matrix
        cols = np.arange(self.Bnnz_k.size)  # 0, 1, ... 63
        Indexer = csc_matrix(
            (data, (rows, cols)), shape=(m * n, self.Bnnz_k.size)
        )  # (rows, cols)	data

        # sparse version instead of the old B_block
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")

        # real data
        self.Bnnz_k.value = B_block.data

        # No need for sparse matrices for C as most values are parameters
        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        # -------------------------------------------------------------
        # TODO: Constraint part 1:
        #       Add dynamics constraints to the optimization problem
        #       This constraint should be based on a few variables:
        #       self.xk, self.Ak_, self.Bk_, self.uk, and self.Ck_

        flatten_prev_xk = cvxpy.vec(self.xk[:, :-1])
        flatten_next_xk = cvxpy.vec(self.xk[:, 1:])
        # flatten_uk = cvxpy.diag(self.uk[:, :-1].flatten())
        # import pdb; pdb.set_trace()
        c1 = (
            flatten_next_xk
            == self.Ak_ @ flatten_prev_xk + self.Bk_ @ cvxpy.vec(self.uk) + self.Ck_
        )
        constraints.append(c1)

        # TODO: Constraint part 2:
        #       Add constraints on steering, change in steering angle
        #       cannot exceed steering angle speed limit. Should be based on:
        #       self.uk, self.config.MAX_DSTEER, self.config.DTK

        dsteering = cvxpy.diff(self.uk[1, :])
        c2_lower = -self.config.MAX_DSTEER * self.config.DTK <= dsteering
        c2_upper = dsteering <= self.config.MAX_DSTEER * self.config.DTK
        # if abs(dsteering).max() > self.config.MAX_DSTEER * self.config.DTK:
        #     self.get_logger().warn(
        #         f"Steering angle change exceeds limit: {abs(dsteering).max()} > {self.config.MAX_DSTEER * self.config.DTK}"
        #     ) ## error
        constraints.append(c2_lower)
        constraints.append(c2_upper)

        # TODO: Constraint part 3:
        #       Add constraints on upper and lower bounds of states and inputs
        #       and initial state constraint, should be based on:
        #       self.xk, self.x0k, self.config.MAX_SPEED, self.config.MIN_SPEED,
        #       self.uk, self.config.MAX_ACCEL, self.config.MAX_STEER

        # init state constraint
        c3 = self.xk[:, 0] == self.x0k
        constraints.append(c3)

        # state consraints
        speed = self.xk[2, :]
        c4_lower = self.config.MIN_SPEED <= speed
        c4_upper = speed <= self.config.MAX_SPEED
        constraints.append(c4_lower)
        constraints.append(c4_upper)

        # input constraints
        steering = self.uk[1, :]
        c5_lower = self.config.MIN_STEER <= steering
        c5_upper = steering <= self.config.MAX_STEER
        constraints.append(c5_lower)
        constraints.append(c5_upper)

        acc = self.uk[0, :]
        c6 = acc <= self.config.MAX_ACCEL
        c7 = -self.config.MAX_ACCEL <= acc
        constraints.append(c6)
        constraints.append(c7)

        # -------------------------------------------------------------

        # Create the optimization problem in CVXPY and setup the workspace
        # Optimization goal: minimize the objective function
        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def nearest_point(self, point, trajectory):
        """
        Return the nearest point along the given piecewise linear trajectory.
        Args:
            point (numpy.ndarray, (2, )): (x, y) of current pose
            trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
                NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
        Returns:
            nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
            nearest_dist (float): distance to the nearest point
            t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
            i (int): index of nearest point in the array of trajectory waypoints
        """
        diffs = trajectory[1:, :] - trajectory[:-1, :]
        l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
        dots = np.empty((trajectory.shape[0] - 1,))
        for i in range(dots.shape[0]):
            dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
        t = dots / l2s
        t[t < 0.0] = 0.0
        t[t > 1.0] = 1.0
        projections = trajectory[:-1, :] + (t * diffs.T).T
        dists = np.empty((projections.shape[0],))
        for i in range(dists.shape[0]):
            temp = point - projections[i]
            dists[i] = np.sqrt(np.sum(temp * temp))
        min_dist_segment = np.argmin(dists)
        return (
            projections[min_dist_segment],
            dists[min_dist_segment],
            t[min_dist_segment],
            min_dist_segment,
        )
    
    def idx_calc(self, x , y):
        idx = self.db.find_idx(x, y, table="path")
        return idx

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp, ind_list):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param cx: Course X-Position
        :param cy: Course y-Position
        :param cyaw: Course Headingtarget_idx
        :param sp: speed profile
        :dl: distance step
        :pind: Setpoint Index
        :return: reference trajectory ref_traj, reference steering angle
        """

        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)

        # Find nearest index from where the trajectories are calculated
        _, dist, _, ind = self.nearest_point(
            np.array([state.x, state.y]), np.array([cx, cy]).T
        )
        # print(dist)
        if dist > 10.: 
            db_idx = self.db.find_idx(state.x, state.y, "Path")
        else:
            db_idx = ind_list[ind]
        # Load the initial parameters from the nearest idx into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]


        # speed_ths_20 = 20.0 /3.6
        # speed_ths_16 = 16.0 / 3.6
        # speed_ths_13 = 13.0 / 3.6

        
        # if sp[ind] >  speed_ths_16:
        #     dind = 6
        # elif sp[ind] > speed_ths_13:
        #     dind = 5
        # else:
        #     dind = 4
        # if state.v < 16.0 / 3.6:
        #     if state.v >  speed_ths_20:
        #         dind = 6
        #     elif state.v > speed_ths_16:
        #         dind = 5
        #     elif state.v > speed_ths_13:
        #         dind = 4
        #     else:
        #         dind = 3

        # v = (state.v + sp[ind]) /2
        # print(v)
        # travel = abs(state.v) * self.config.DTK
        # dind =  ( travel / self.config.dlk)
        dind = 6  # 기본값
        try: 
            if self.last_pred_path is not None:
                diffs = np.diff(self.last_pred_path[0:2, :], axis=1)  # (x,y) 차이
                seg_lengths = np.sqrt(np.sum(diffs**2, axis=0))
                total_length = np.sum(seg_lengths)
                ind_total_length = 10 * total_length
                margin = 10 #1m  # 필요시 조정
                dind = (ind_total_length + margin) / max(1, self.config.TK)
                # self.get_logger().info(f"pred path length={total_length:.2f}, dind={dind:.2f}")
        except Exception as e:
            self.get_logger().warn(f"[MPC] dind from last_pred_path failed: {e}")
        # try:
        #     if self.last_pred_position[0]:  # 0이면 False
        #         _, _, _, ind_last = self.nearest_point(
        #             np.array([self.last_pred_position[0], self.last_pred_position[1]]), np.array([cx, cy]).T
        #         )
        #         if 0 <= ind_last < ncourse:
        #             ind_length = abs(int(ind) - ind_last) + 10  # margin
        #             dind = ind_length / max(1, self.config.TK)
        #             self.get_logger().info(f'length: {ind_length}, dind : {dind}')
        # except Exception as e:
        #     self.get_logger().warn(f"[MPC] dind from last_pred_position failed: {e}")

        # 남아있는 idx 개수에서 내 위치 뺀 것과 1 중에서 큰 값 결정, 최소 1 확보
        rest_idx_num = max(len(cx) - ind - 1, 1)

        # 남아있는 idx 개수를 예측하고 싶은 horizon으로 나눠 각 스텝 간 최대 간격 계산
        max_dind = int(rest_idx_num / self.config.TK)

        # 계산된 최대 간격과 1을 비교해서 가장 큰 값을 결정하고, 그 값을 지정해놓은 간격과 비교해서 가장 작은 값을 결정
        dind = min(dind, max(1, max_dind))
        # 최소 3 확보
        dind = max(1, dind) 
        # [dind, dind, ... , dind] TK개  [dind, 2*dind, ... , TK*dind]  [0, dind, ... , TK*dind] - 내 위치 추가
        ind_offsets = np.insert(np.cumsum([dind] * self.config.TK), 0, 0)

        # reference trajectory index list
        ind_list = np.clip(int(ind) + ind_offsets, 0, len(cx) - 1).astype(int)

        # reference trajectory가 비현실적인 idx를 가지면 마지막 인덱스로 고정
        ind_list[ind_list >= ncourse] = ncourse - 1

        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
            

        angle_thres = 4.5

        for i in range(len(cyaw)):
            if cyaw[i] - state.yaw > angle_thres:
                cyaw[i] -= 2 * np.pi
            if state.yaw - cyaw[i] > angle_thres:
                cyaw[i] += 2 * np.pi

        ref_traj[3, :] = cyaw[ind_list]

        return ref_traj, ind, db_idx

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for ai, di, i in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state, a_cmd, delta_cmd):
        
        if delta_cmd >= self.config.MAX_STEER:
            delta_cmd = self.config.MAX_STEER
        elif delta_cmd <= -self.config.MAX_STEER:
            delta_cmd = -self.config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw = (
            state.yaw
            + (state.v / self.config.WB) * math.tan(delta_cmd) * self.config.DTK
        )
        state.v = state.v + a_cmd * self.config.DTK

        # 속도 제한
        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED

        return state

    def get_model_matrix(self, v, phi, delta):
        """
        Calc linear and discrete time dynamic model-> Explicit discrete time-invariant
        Linear System: Xdot = Ax +Bu + C
        State vector: x=[x, y, v, yaw]
        :param v: speed
        :param phi: heading angle of the vehicle
        :param delta: steering angle: delta_bar
        :return: A, B, C

        Calc linear and discrete time dynamic model with first-order delay
        for steering and velocity.
        State vector: x=[x, y, v, yaw]
        Input vector: u=[accel_cmd, steer_cmd]
        """
        
        # State (or system) matrix A, 4x4
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        # Input Matrix B; 4x2
        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C  # 4 x 4, 4 x 2, 4 x 1

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem in CVXPY
        # Solver selections: cvxpy.OSQP; cvxpy.GUROBI
        try:
            # self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)
            # default max_iter = 4000 eps_abs = 1e-3 eps_rel = 1e-3
            # light solver settings max_iter = 1500, eps_abs = 3e-3, eps_rel = 3e-3
            self.MPC_prob.solve(
                solver=cvxpy.OSQP, verbose=False, warm_start=not self.reset_ws
            )
            if self.reset_ws:
                self.reset_ws = False
        except Exception as e:
            print(f"[MPC] Solve failed with exception: {e}")
            return None, None, None, None, None, None

        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()
            dat = 0  # success
        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None
            dat = 1  # failure

        self.pub_error.publish(Int32(data=dat))
        return oa, odelta, ox, oy, oyaw, ov

    def linear_mpc_control(self, ref_path, x0, oa, od):
        """
        MPC control with updating operational point iteraitvely
        :param ref_path: reference trajectory in T steps
        :param x0: initial state vector
        :param oa: acceleration of T steps of last time
        :param od: delta of T steps of last time
        """

        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        # Call the Motion Prediction function: Predict the vehicle motion for x-steps
        path_predict = self.predict_motion(x0, oa, od, ref_path)
        self.last_pred_path = path_predict
        self.visualize_pred_path_in_rviz(path_predict)

        ########################################## 연산 오래걸림 ##########################################
        # Run the MPC optimization: Create and solve the optimization problem
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0
        )
        ########################################## 연산 오래걸림 ##########################################

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict
        
    # visualization
    def visualize_waypoints_in_rviz(self):
        self.vis_waypoints_msg.points = []
        self.vis_waypoints_msg.header.frame_id = "/map"
        self.vis_waypoints_msg.type = Marker.POINTS
        self.vis_waypoints_msg.color.g = 0.75
        self.vis_waypoints_msg.color.a = 1.0
        self.vis_waypoints_msg.scale.x = 0.05
        self.vis_waypoints_msg.scale.y = 0.05
        self.vis_waypoints_msg.id = 0
        for i in range(self.waypoints.shape[1]):
            point = Point(x=self.waypoints[0, i], y=self.waypoints[1, i], z=0.1)
            self.vis_waypoints_msg.points.append(point)

        self.vis_waypoints_pub.publish(self.vis_waypoints_msg)

    def visualize_ref_traj_in_rviz(self, ref_traj):
        pa = PoseArray()
        pa.header.frame_id = "/map"
        pa.poses = []

        # ref_traj: [x, y, v, yaw] 형태
        for i in range(ref_traj.shape[1]):
            pose = Pose()
            pose.position.x = float(ref_traj[0, i])
            pose.position.y = float(ref_traj[1, i])
            pose.position.z = 0.2  # 기존 마커 z와 유사하게 표시

            yaw = float(ref_traj[3, i])
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)

            pa.poses.append(pose)

        # PoseArray 퍼블리시
        self.vis_ref_traj_pub.publish(pa)

    def visualize_pred_path_in_rviz(self, path_predict):
        pa = PoseArray()
        pa.header.frame_id = "/map"
        pa.poses = []

        # path_predict: [x, y, v, yaw] 형태
        for i in range(path_predict.shape[1]):
            pose = Pose()
            pose.position.x = float(path_predict[0, i])
            pose.position.y = float(path_predict[1, i])
            pose.position.z = 0.2

            yaw = float(path_predict[3, i])
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)

            pa.poses.append(pose)

        # PoseArray 퍼블리시
        self.vis_pred_path_pub.publish(pa)

def main(args=None):
    file_name = "mpc_ys_smooth_new.db"
    db = DB(file_name)
    rclpy.init(args=args)
    print("MPC Initialized")
    mpc_node = MPC(db)
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()