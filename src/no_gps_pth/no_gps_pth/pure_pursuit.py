#!/usr/bin/env python3

import rclpy
import numpy as np
import math as m
from erp42_msgs.msg import StanleyError,SerialFeedBack
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32





class PurePursuit(Node):
    def __init__(self):
        super().__init__("stanley")
        self.__L = 1.240  # [m] Wheel base of vehicle
        # self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.8).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.03).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.06).value

        
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.5).value
        # self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.24).value

        #self.__hdr = 0.0    #heading error
        #self.__ctr = 0.0    #crosstrack error
        
        #self.k_v = 0.5
        self.old_nearest_point_index = None
        self.k = 0.0 #look up distance를 다양하게 조정
        self.Lfc = 4.5


        
    # def stanley_control(self, state, cx, cy, cyaw, last_target_idx, reverse=False):
    def pure_pursuit_control(self, state, cx, cy, reverse=False):

        current_target_idx, Lf = self.calc_target_index(state, cx, cy, reverse=reverse)

        if current_target_idx < len(cx):
            tx = cx[current_target_idx]
            ty = cy[current_target_idx]
        else:  # toward goal
            tx = cx[-1]
            ty = cy[-1]
            current_target_idx = len(cx) - 1


        alpha = m.atan2(ty - self.fy, tx - self.fx) - state.yaw
        
        delta = m.atan2(2 * self.__L * m.sin(alpha) / Lf, 1.0) * (-1.0 if reverse else 1.0)
        print(delta)
        

        delta = (np.clip(delta, m.radians((-1) * 28), m.radians(28)))
        print(delta)

        return delta, current_target_idx


    def calc_target_index(self, state, cx, cy, reverse=False):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """

        self.fx = state.x - self.__L * \
            np.cos(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)
        self.fy = state.y - self.__L * \
            np.sin(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)
        
        if self.old_nearest_point_index is None:
            # Search nearest point index
            dx = [self.fx - icx for icx in cx]
            dy = [self.fy - icy for icy in cy]

            d = np.hypot(dx, dy)
            target_idx = int(np.argmin(d))
            self.old_nearest_point_index = target_idx
        else:
            target_idx = self.old_nearest_point_index
            distance_this_index = self.calc_distance(cx[target_idx],
                                                      cy[target_idx])
            while True:
                distance_next_index = self.calc_distance(cx[target_idx + 1],
                                                          cy[target_idx + 1])
                if distance_this_index < distance_next_index:
                    break
                target_idx = target_idx + 1 if (target_idx + 1) < len(self.cx) else target_idx
                distance_this_index = distance_next_index
            self.old_nearest_point_index = target_idx
        
        Lf = self.k*state.v + self.Lfc

        while Lf > self.calc_distance(cx[target_idx], cy[target_idx]):
            if (target_idx + 1) >= len(cx):
                break  # not exceed goal
            target_idx += 1

        return target_idx, Lf #target_idx , Lf여야함 원래
    
    def calc_distance(self, point_x, point_y):
        dx = self.fx - point_x
        dy = self.fy - point_y
        return m.hypot(dx, dy)
