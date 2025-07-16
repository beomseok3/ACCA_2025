#!/usr/bin/env python3

import time
import cvxpy
import sys
import pathlib
import rclpy
import numpy as np
import math 
import matplotlib.pyplot as plt
from erp42_msgs.msg import StanleyError,SerialFeedBack
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

show_animation = False

'''
class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
'''

class ILMPC(Node):
    def __init__(self):   
        super().__init__('ILMPC')
        self.__L = 1.040
        self.NX = 4 # x = x, y, v, yaw
        self.NU = 2 # a = [accel,steer]
        self.T = 5 # horizon length

        #mpc parameters
        self.R = np.diag([0.01, 0.01]) # input cost matrix
        self.Rd = np.diag([0.01, 1.0]) # input difference cost matrix

        # R, Rd, Q, Qf에 가중치에 따라 뭐가 더 중요하게 판단할 지 정할 수 있음 가령, Rd 의 0.01은 accel, 1.0은 steer값임. 해당 값을 통해 steer를 확 틀지 않게 할 수 있음
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5]) #state cost matrix    
        self.Qf = self.Q # state final matrix
        self.GOAL_DIS = 1.5 # goal distance
        self.STOP_SPEED = 0.5 / 3.6 # stop speed
        self.MAX_TIME= 500.0 # max simualtion time

        # iterative parameter
        self.MAX_ITER = 3 # Max iteration
        self.DU_TH = 0.1 # iteration finish param

        self.TARGET_SPEED = 10.0 / 3.6 # [m/s] target speed
        self.N_IND_SEARCH = 10 # Search index number

        self.DT = 0.2 # [s] time tick

        # Vehicle parameters
        self.LENGTH = 4.5 # [m]
        self.WIDTH = 2.0 # [m]
        self.BACKTOWHEEL = 1.0 # [m]
        self.WHEEL_LEN = 0.3 # [m]
        self.WHELL_WIDTH = 0.2 # [m]
        self.TREAD = 0.7 # [m]
        self.WB = 1.040 # [m]

        self.MAX_STEER = np.deg2rad(28.0)  # maximum steering angle [rad]
        self.MAX_DSTEER = np.deg2rad(10.0)  # maximum steering speed [rad/s]
        self.MAX_SPEED = 20.0 / 3.6  # maximum speed [m/s]
        self.MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
        self.MAX_ACCEL = 1.0  # maximum accel [m/ss]show_animation = True

        #pi_2_pi 즉 normailize_angle 함수를 실행하기 위한 함수
    def angle_mod(self, x, zero_2_2pi=False, degree=False): 
        """
        Angle modulo operation
        Default angle modulo range is [-pi, pi)

        Parameters
        ----------
        x : float or array_like
            A angle or an array of angles. This array is flattened for
            the calculation. When an angle is provided, a float angle is returned.
        zero_2_2pi : bool, optional
            Change angle modulo range to [0, 2pi)
            Default is False.
        degree : bool, optional
            If True, then the given angles are assumed to be in degrees.
            Default is False.

        Returns
        -------
        ret : float or ndarray
            an angle or an array of modulated angle.

        Examples
        --------
        >>> angle_mod(-4.0)
        2.28318531

        >>> angle_mod([-4.0])
        np.array(2.28318531)

        >>> angle_mod([-150.0, 190.0, 350], degree=True)
        array([-150., -170.,  -10.])

        >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
        array([300.])

        """
        if isinstance(x, float):
        
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle

    def normalize_angle(self, angle): # 
        """
        angle_mod   범위를 -pi <   < pi로 
        
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """'''
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi
        '''
        return self.angle_mod(angle)

    def get_linear_model_matrix(self, v, phi, delta): 
        '''모델을 표현하는 행렬
        A = 운동학적 모델
        B = 제어 입력이 미치는 영향
        C = 비선형항 보정'''

        A = np.zeros((self.NX, self.NX))  
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.DT * math.cos(phi)
        A[0, 3] = - self.DT * v * math.sin(phi)
        A[1, 2] = self.DT * math.sin(phi)
        A[1, 3] = self.DT * v * math.cos(phi)
        A[3, 2] = self.DT * math.tan(delta) / self.WB

        B = np.zeros((self.NX, self.NU)) 
        B[2, 0] = self.DT
        B[3, 1] = self.DT * v / (self.WB * math.cos(delta) ** 2)

        C = np.zeros(self.NX) 
        C[0] = self.DT * v * math.sin(phi) * phi
        C[1] = - self.DT * v * math.cos(phi) * phi
        C[3] = - self.DT * v * delta / (self.WB * math.cos(delta) ** 2)

        return A, B, C
    
    
    def update_state(self, state, a, delta):
        '''
        상태 업데이트 및 최고 속력, 최소 속력 정하는 값
        PID를 통한 속도 제어를 할 수도 있고 정할 수 있음
        '''
        # input check
        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.DT
        state.y = state.y + state.v * math.sin(state.yaw) * self.DT
        state.yaw = state.yaw + state.v / self.WB * math.tan(delta) * self.DT
        state.v = state.v + a * self.DT

        if state.v > self.MAX_STEER:
            state.v = self.MAX_SPEED
        elif state.v < self.MIN_SPEED:
            state.v = self.MIN_SPEED
        
        return state
    

    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()


    def calc_nearest_index(self, state, cx, cy, cyaw, pind): 


        dx = [state.x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.normalize_angle(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1
        
        return ind, mind
    
    def predict_motion(self, state, x0, oa, od, xref):  
        '''
        예측값 -> 측정값과 예측값을 통한 수정이 이루어지는 게 MPC기에.
        '''
        
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        for (ai, di, i) in zip(oa, od, range(1, self.T + 1)):
            state = self.update_state(state, ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar
    
    
    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):  
        '''
        비선형을 선형근사 시키는 함수
        '''

        ox, oy, oyaw, ov = None, None, None, None

        if oa is None or od is None:
            oa = [0.0] * self.T
            od = [0,0] * self.T

        for i in range(self.MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))
            if du <= self.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov
    

    def linear_mpc_control(self, xref, xbar, x0, dref):   #선형MPC로서 할 수 있게끔

        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """
        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))

        cost = 0.0
        constraints = [] # 제약조건.

        for t in range(self.T): # quad_form함수를 통한 비용함수 << 최적화 문제랑 연결
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= self.MAX_DSTEER * self.DT ]

        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= self.MAX_SPEED]
        constraints += [x[2, :] >= self.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER] #제약 추가

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver = cvxpy.CLARABEL, verbose=False) #최적화 문제를 CLARABEL 솔버

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:  # 최적화 시 호출
            ox = self.get_nparray_from_matrix(x.value[0, :])
            oy = self.get_nparray_from_matrix(x.value[1, :])
            ov = self.get_nparray_from_matrix(x.value[2, :])
            oyaw = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:  #최적화 안 될 경우 None값 반환
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp, dl, pind):  #참조값 계산
        xref = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((1, self.T + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0 # steer operational point should be 0

        travel = 0.0

        for i in range(self.T + 1):
            travel += abs(state.v) * self.DT
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref

    def check_goal(self, state, goal, tind, nind): #췤

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.hypot(dx, dy)

        isgoal = (d <= self.GOAL_DIS)

        if abs(tind - nind) >= 5:
            isgoal = False
        
        isstop = (abs(state.v) <= self.STOP_SPEED)

        if isgoal and isstop:
            return True
        
        return False


    def do_simulation(self, state, cx, cy, cyaw):  # 메인 함수로서 사용
        dl = 1.0 

        sp = self.calc_speed_profile(cx, cy, cyaw, self.TARGET_SPEED)

        #initial_state = state(x= state.x, y= state.y, yaw=state.yaw, v=state.v)
                                #(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

        """
        Simulation

        cx: course x position list
        cy: course y position list
        cy: course yaw position list
        ck: course curvature list
        sp: speed profile
        dl: course tick [m]

        """

        goal = [cx[-1], cy[-1]]

        #state = initial_state

        # initial yaw compensation
        if state.yaw - cyaw[0] >= math.pi:
            state.yaw -= math.pi * 2.0
        elif state.yaw - cyaw[0] <= -math.pi:
            state.yaw += math.pi * 2.0

        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, 0)

        odelta, oa = None, None

        cyaw = self.smooth_yaw(cyaw)

        xref, target_ind, dref = self.calc_ref_trajectory(
            state, cx, cy, cyaw, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        di, ai = 0.0, 0.0
        if odelta is not None:
            di, ai = odelta[0], oa[0]
            state = self.update_state(state, ai, di)

        time = time + self.DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)


        return t, x, y, yaw, v, d, a , target_ind

    
    
    def calc_speed_profile(self, cx, cy, cyaw, target_speed): 

        speed_profile = [target_speed] * len(cx)
        direction = 1.0 # forward

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(self.normalize_angle(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0

        return speed_profile

    def smooth_yaw(self, yaw):

        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw

'''
def main(args=None):
    rclpy.init(args=args)
    print(__file__ + " start!!")
    start = time.time()
    a = ILMPC()

    dl = 1.0  # course tick
    # cx, cy, cyaw, ck = get_straight_course(dl)
    # cx, cy, cyaw, ck = get_straight_course2(dl)
    # cx, cy, cyaw, ck = get_straight_course3(dl)
    # cx, cy, cyaw, ck = get_forward_course(dl)


    #cx, cy, cyaw, ck = 여기에 무언가를 해야 됑
    
    sp = a.calc_speed_profile(cx, cy, cyaw, a.TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    t, x, y, yaw, v, d, a = a.do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    elapsed_time = time.time() - start
    print(f"calc time:{elapsed_time:.6f} [sec]")
'''