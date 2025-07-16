#!/usr/bin/env python3

import rclpy
import numpy as np
from erp42_msgs.msg import StanleyError
from rclpy.node import Node
from rclpy.qos import QoSProfile


"""

Export module. Stanley Control Class.
Input: (state(class: State), [cx], [cy], [cyaw], last_target_idx)
Output: steer

"""

class Stanley(Node):
    def __init__(self):
        super().__init__("stanley")
        qos_profile = QoSProfile(depth = 1)

        self.__L = 1.040  # [m] Wheel base of vehicle
        self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.8).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.03).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.06).value
        self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.5).value

        self.__ind = 0

        self.__hdr = 0.0    #heading error
        self.__ctr = 0.0    #crosstrack error
        
        self.k_v = 1.1

        self.__error_publisher = self.create_publisher(StanleyError, "stanley_error", qos_profile)

    def resetIdx(self):
        self.__ind = 0
    
    def getCGain(self):
        return self.__k
    
    def setCGain(self, value):
        self.__k = value
        print("SET C GAIN")
        return self.__k
    
    def setHdrRatio(self, value):
        self.__hdr_ratio = value
        return self.__hdr_ratio
    
    def stanley_control(self, state, cx, cy, cyaw, last_target_idx, reverse=False):
        current_target_idx, error_front_axle = self.calc_target_index(
        state, cx, cy, reverse=reverse)


        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = (self.normalize_angle(
            cyaw[current_target_idx] - (state.yaw + (np.pi if reverse else 0.)))) * self.__hdr_ratio
        

        # theta_d corrects the cross track error
        # theta_d = np.arctan2(self.__k * error_front_axle,
        #                     state.v) * (-1.0 if reverse else 1.0)
        
        theta_d = np.arctan2(self.__k * error_front_axle,
                            (self.k_v + state.v)) * (-1.0 if reverse else 1.0)


        # Field
        self.__hdr = theta_e
        self.__ctr = theta_d

        # Steering control
        delta = theta_e + theta_d

        # if self.__error_publisher.get_subscription_count() > 0:
        #     msg = StanleyError()
        #     msg.hdr = self.__hdr
        #     msg.ctr = self.__ctr
        #     self.__error_publisher.publish(msg)

        # self.__ind = current_target_idx

        # print("heading err: {}, cross track err : {}, hdr: {}, ctr:{}".format((theta_e / self.__hdr_ratio), error_front_axle, self.__hdr, self.__ctr))

        return delta, current_target_idx, self.__hdr, self.__ctr, (theta_e / self.__hdr_ratio), error_front_axle
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def calc_target_index(self, state, cx, cy, reverse=False):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position

        fx = state.x + self.__L * \
            np.cos(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)
        fy = state.y + self.__L * \
            np.sin(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)


        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]

        d = np.hypot(dx, dy)
        target_idx = int(np.argmin(d))

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]

        error_front_axle = np.dot(
            [dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
    