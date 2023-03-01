"""
Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı
"""

import math
from enum import Enum
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import imageio

from rclpy.node import Node
import rclpy


show_animation = True
ims = []

def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.26  # [m/s]
        self.min_speed = -0.26  # [m/s]
        self.max_yaw_rate =  	1.82  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate =  	1.82  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1 # [s] Time tick for motion prediction
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 10
        self.obstacle_cost_gain = 1
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.306  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([[ 0.5 , -0.01],
                            [ 0.51, -0.02],
                            [ 0.51, -0.02],
                            [ 0.51, -0.05],
                            [ 0.51, -0.06],
                            [ 0.51, -0.29],
                            [ 0.5 , -0.31],
                            [ 0.5 , -0.32],
                            [ 0.5 , -0.32],
                            [-0.42, -0.32],
                            [-0.43, -0.31],
                            [-0.43, -0.3 ],
                            [-0.43, -0.3 ],
                            [-0.43, -0.05],
                            [-0.43, -0.02],
                            [-0.43,  0.  ],
                            [-0.43,  0.  ],
                            [-0.43,  0.01],
                            [-0.43,  0.01],
                            [-0.42,  0.27],
                            [-0.4 ,  0.29],
                            [-0.4 ,  0.3 ],
                            [-0.4 ,  0.31],
                            [-0.4 ,  0.31],
                            [-0.4 ,  0.32],
                            [-0.4 ,  0.33],
                            [-0.41,  0.33],
                            [-0.43,  0.33],
                            [-0.43,  0.33],
                            [-0.43,  0.33],
                            [-0.38,  0.58],
                            [-0.38,  0.58],
                            [-0.37,  0.59],
                            [-0.35,  0.59],
                            [-0.35,  0.59],
                            [-0.34,  0.6 ],
                            [-0.34,  0.61],
                            [-0.34,  0.61],
                            [-0.34,  0.63],
                            [-0.29,  0.86],
                            [-0.28,  0.87],
                            [-0.27,  0.87],
                            [-0.27,  0.87],
                            [-0.26,  0.88],
                            [-0.25,  0.89],
                            [-0.25,  0.9 ],
                            [-0.18 , 1.14],
                            [-0.17 , 1.14],
                            [-0.15 , 1.14],
                            [-0.14 , 1.15],
                            [-0.14 , 1.19],
                            [-0.05 , 1.38],
                            [-0.04 , 1.4 ],
                            [-0.02 , 1.4 ],
                            [-0.   , 1.4 ],
                            [ 0.66 , 1.08],
                            [ 0.67 , 1.07],
                            [ 0.67 , 1.05],
                            [ 0.67 , 1.05],
                            [ 0.55,  0.81],
                            [ 0.55,  0.81],
                            [ 0.55,  0.79],
                            [ 0.55,  0.78],
                            [ 0.55,  0.78],
                            [ 0.55,  0.78],
                            [ 0.51,  0.57],
                            [ 0.5 ,  0.57],
                            [ 0.5 ,  0.55],
                            [ 0.5 ,  0.55],
                            [ 0.5 ,  0.54],
                            [ 0.51,  0.53],
                            [ 0.51,  0.52],
                            [ 0.52,  0.32],
                            [ 0.52,  0.31],
                            [ 0.52,  0.31],
                            [ 0.51,  0.3 ],
                            [ 0.51,  0.29],
                            [ 0.51,  0.29],
                            [ 0.51,  0.27],
                            [ 0.51, -0.01],
                            [ 0.51, -0.02],
                            [ 0.51, -0.03],
                            [ 0.51, -0.04],
                            [ 0.51, -0.04],
                            [ 0.5 , -0.31],
                            [ 0.5 , -0.31],
                            [ 0.5 , -0.32],
                            [ 0.49, -0.33],
                            [-0.43, -0.33],
                            [-0.44, -0.33],
                            [-0.44, -0.33],
                            [-0.44, -0.3 ],
                            [-0.42, -0.02],
                            [-0.42, -0.02],
                            [-0.42,  0.01],
                            [-0.43,  0.01],
                            [-0.43,  0.01],
                            [-0.43,  0.01],
                            [-0.41,  0.28],
                            [-0.39,  0.29],
                            [-0.39,  0.3 ],
                            [-0.39,  0.31],
                            [-0.4 ,  0.31],
                            [-0.4 ,  0.32],
                            [-0.41,  0.32],
                            [-0.41,  0.32],
                            [-0.43,  0.32],
                            [-0.43,  0.33],
                            [-0.41,  0.34],
                            [-0.38,  0.57],
                            [-0.38,  0.59],
                            [-0.37,  0.59],
                            [-0.36,  0.59],
                            [-0.36,  0.6 ],
                            [-0.35,  0.6 ],
                            [-0.36,  0.61],
                            [-0.35,  0.62],
                            [-0.35,  0.63],
                            [-0.3 ,  0.85],
                            [-0.27,  0.86],
                            [-0.27,  0.86],
                            [-0.26,  0.86],
                            [-0.25,  0.87],
                            [-0.24,  0.87],
                            [-0.24,  0.89],
                            [-0.18 , 1.14],
                            [-0.17 , 1.14],
                            [-0.16 , 1.15],
                            [-0.14 , 1.15],
                            [-0.13 , 1.18],
                            [-0.05 , 1.38],
                            [-0.04 , 1.39],
                            [-0.02 , 1.39],
                            [-0.   , 1.38],
                            [ 0.66 , 1.08],
                            [ 0.67 , 1.07],
                            [ 0.67 , 1.06],
                            [ 0.67 , 1.05],
                            [ 0.55,  0.81],
                            [ 0.55,  0.81],
                            [ 0.55,  0.8 ],
                            [ 0.56,  0.79],
                            [ 0.56,  0.78],
                            [ 0.56,  0.78],
                            [ 0.51,  0.57],
                            [ 0.51,  0.56],
                            [ 0.5 ,  0.56],
                            [ 0.51,  0.55],
                            [ 0.51,  0.53],
                            [ 0.51,  0.53],
                            [ 0.51,  0.52],
                            [ 0.51,  0.32],
                            [ 0.51,  0.32],
                            [ 0.51,  0.31],
                            [ 0.51,  0.3 ],
                            [ 0.51,  0.29],
                            [ 0.52,  0.28],
                            [ 0.52,  0.26],
                            [ 0.52, -0.01],
                            [ 0.52, -0.02],
                            [ 0.51, -0.02],
                            [ 0.51, -0.04],
                            [ 0.5 , -0.04],
                            [ 0.5 , -0.29],
                            [ 0.5 , -0.3 ],
                            [ 0.5 , -0.32],
                            [ 0.5 , -0.32],
                            [-0.42, -0.33],
                            [-0.43, -0.33],
                            [-0.45, -0.32],
                            [-0.45, -0.32],
                            [-0.45, -0.3 ],
                            [-0.43, -0.02],
                            [-0.43, -0.01],
                            [-0.43, -0.01],
                            [-0.43, -0.  ],
                            [-0.43,  0.01],
                            [-0.43,  0.01],
                            [-0.41,  0.28],
                            [-0.4 ,  0.3 ],
                            [-0.4 ,  0.3 ],
                            [-0.4 ,  0.3 ],
                            [-0.4 ,  0.31],
                            [-0.4 ,  0.32],
                            [-0.4 ,  0.33],
                            [-0.41,  0.33],
                            [-0.42,  0.33],
                            [-0.42,  0.33],
                            [-0.42,  0.33],
                            [-0.38,  0.58],
                            [-0.37,  0.58],
                            [-0.36,  0.59],
                            [-0.36,  0.59],
                            [-0.35,  0.59],
                            [-0.35,  0.6 ],
                            [-0.36,  0.61],
                            [-0.36,  0.62],
                            [-0.35,  0.63],
                            [-0.28,  0.86],
                            [-0.28,  0.86],
                            [-0.27,  0.86],
                            [-0.26,  0.86],
                            [-0.25,  0.87],
                            [-0.25,  0.88],
                            [-0.24,  0.89],
                            [-0.18 , 1.14],
                            [-0.17 , 1.15],
                            [-0.15 , 1.16],
                            [-0.14 , 1.16],
                            [-0.05 , 1.39],
                            [-0.03 , 1.4 ],
                            [-0.02 , 1.4 ],
                            [-0.   , 1.39],
                            [ 0.68 , 1.09],
                            [ 0.68 , 1.07],
                            [ 0.68 , 1.05],
                            [ 0.68 , 1.04],
                            [ 0.56,  0.83],
                            [ 0.55,  0.8 ],
                            [ 0.55,  0.79],
                            [ 0.55,  0.78],
                            [ 0.55,  0.78],
                            [ 0.51,  0.57],
                            [ 0.51,  0.56],
                            [ 0.5 ,  0.56],
                            [ 0.51,  0.54],
                            [ 0.51,  0.54],
                            [ 0.51,  0.53],
                            [ 0.51,  0.52],
                            [ 0.51,  0.33],
                            [ 0.51,  0.32],
                            [ 0.51,  0.31],
                            [ 0.51,  0.3 ],
                            [ 0.51,  0.29],
                            [ 0.52,  0.28],
                            [ 0.51,  0.26],
                            [ 0.51, -0.  ],
                            [ 0.5 , -0.01],
                            [ 0.5 , -0.02],
                            [ 0.5 , -0.02],
                            [ 0.5 , -0.04],
                            [ 0.5 , -0.29],
                            [ 0.5 , -0.3 ],
                            [ 0.5 , -0.31],
                            [ 0.5 , -0.31],
                            [-0.42, -0.32],
                            [-0.42, -0.32],
                            [-0.44, -0.32],
                            [-0.44, -0.31],
                            [-0.44, -0.3 ],
                            [-0.43, -0.04],
                            [-0.43, -0.01],
                            [-0.43,  0.01],
                            [-0.43,  0.01],
                            [-0.43,  0.01],
                            [-0.43,  0.02],
                            [-0.41,  0.28],
                            [-0.41,  0.29],
                            [-0.4 ,  0.3 ],
                            [-0.4 ,  0.31],
                            [-0.4 ,  0.31],
                            [-0.4 ,  0.32],
                            [-0.41,  0.33],
                            [-0.42,  0.33],
                            [-0.42,  0.33],
                            [-0.42,  0.34],
                            [-0.39,  0.58],
                            [-0.38,  0.58],
                            [-0.36,  0.59],
                            [-0.36,  0.59],
                            [-0.35,  0.59],
                            [-0.35,  0.6 ],
                            [-0.35,  0.61],
                            [-0.35,  0.61],
                            [-0.35,  0.63],
                            [-0.29,  0.86],
                            [-0.28,  0.86],
                            [-0.27,  0.86],
                            [-0.26,  0.86],
                            [-0.25,  0.87],
                            [-0.25,  0.88],
                            [-0.25,  0.9 ],
                            [-0.2  , 1.15],
                            [-0.18 , 1.15],
                            [-0.16 , 1.16],
                            [-0.15 , 1.16],
                            [-0.14 , 1.16],
                            [-0.05 , 1.38],
                            [-0.03 , 1.38],
                            [-0.01 , 1.38],
                            [ 0.67 , 1.08],
                            [ 0.68 , 1.06],
                            [ 0.68 , 1.05],
                            [ 0.68 , 1.04],
                            [ 0.55,  0.83],
                            [ 0.55,  0.81],
                            [ 0.55,  0.8 ],
                            [ 0.55,  0.79],
                            [ 0.55,  0.78],
                            [ 0.55,  0.78],
                            [ 0.51,  0.57],
                            [ 0.51,  0.56],
                            [ 0.51,  0.55],
                            [ 0.51,  0.54],
                            [ 0.51,  0.53],
                            [ 0.51,  0.53],
                            [ 0.51,  0.52],
                            [ 0.52,  0.32],
                            [ 0.52,  0.32],
                            [ 0.51,  0.3 ],
                            [ 0.51,  0.3 ],
                            [ 0.51,  0.29],
                            [ 0.51,  0.28],
                            [ 0.52,  0.26],
                            [ 0.52, -0.01],
                            [ 0.52, -0.01],
                            [ 0.5 , -0.02],
                            [ 0.5 , -0.03],
                            [ 0.5 , -0.04],
                            [ 0.5 , -0.31],
                            [ 0.5 , -0.31],
                            [ 0.5 , -0.32],
                            [ 0.5 , -0.33],
                            [-0.43, -0.33],
                            [-0.44, -0.33],
                            [-0.45, -0.32],
                            [-0.45, -0.32],
                            [-0.45, -0.31],
                            [-0.44, -0.04],
                            [-0.44, -0.02],
                            [-0.44, -0.01],
                            [-0.44, -0.  ],
                            [-0.44,  0.01],
                            [-0.44,  0.01],
                            [-0.41,  0.28],
                            [-0.41,  0.29],
                            [-0.41,  0.3 ],
                            [-0.41,  0.31],
                            [-0.4 ,  0.32],
                            [-0.4 ,  0.33],
                            [-0.41,  0.33],
                            [-0.4 ,  0.33],
                            [-0.4 ,  0.34],
                            [-0.39,  0.58],
                            [-0.37,  0.58],
                            [-0.36,  0.58],
                            [-0.36,  0.59],
                            [-0.36,  0.6 ],
                            [-0.36,  0.61],
                            [-0.36,  0.61],
                            [-0.36,  0.62],
                            [-0.36,  0.63],
                            [-0.28,  0.85],
                            [-0.28,  0.87],
                            [-0.27,  0.87],
                            [-0.26,  0.87],
                            [-0.25,  0.88],
                            [-0.25,  0.88],
                            [-0.24,  0.89],
                            [-0.18 , 1.14],
                            [-0.18 , 1.15],
                            [-0.16 , 1.15],
                            [-0.15 , 1.16],
                            [-0.14 , 1.16],
                            [-0.06 , 1.38],
                            [-0.04 , 1.4 ],
                            [-0.03 , 1.4 ],
                            [-0.01 , 1.38],
                            [ 0.65 , 1.1 ],
                            [ 0.68 , 1.06],
                            [ 0.68 , 1.05],
                            [ 0.65 , 1.04],
                            [ 0.55,  0.83],
                            [ 0.55,  0.8 ],
                            [ 0.55,  0.8 ],
                            [ 0.55,  0.79],
                            [ 0.55,  0.78],
                            [ 0.54,  0.77],
                            [ 0.5 ,  0.57],
                            [ 0.5 ,  0.56],
                            [ 0.5 ,  0.56],
                            [ 0.51,  0.55],
                            [ 0.51,  0.54],
                            [ 0.51,  0.53],
                            [ 0.52,  0.52],
                            [ 0.52,  0.32],
                            [ 0.52,  0.32],
                            [ 0.51,  0.31],
                            [ 0.51,  0.3 ],
                            [ 0.51,  0.29],
                            [ 0.51,  0.28],
                            [ 0.52,  0.28],
                            [ 0.52, -0.01],
                            [ 0.51, -0.01],
                            [ 0.5 , -0.02],
                            [ 0.5 , -0.03],
                            [ 0.5 , -0.05],
                            [ 0.5 , -0.3 ],
                            [ 0.5 , -0.3 ],
                            [ 0.5 , -0.31],
                            [ 0.49, -0.32],
                            [ 0.49, -0.32],
                            [-0.42, -0.32],
                            [-0.43, -0.31],
                            [-0.44, -0.31],
                            [-0.45, -0.31],
                            [-0.45, -0.3 ],
                            [-0.45, -0.3 ],
                            [-0.43, -0.02],
                            [-0.43, -0.  ],
                            [-0.43, -0.  ],
                            [-0.43,  0.  ],
                            [-0.43,  0.01],
                            [-0.43,  0.02],
                            [-0.42,  0.27],
                            [-0.4 ,  0.29],
                            [-0.4 ,  0.3 ],
                            [-0.4 ,  0.31],
                            [-0.4 ,  0.32],
                            [-0.4 ,  0.32],
                            [-0.41,  0.33],
                            [-0.41,  0.33],
                            [-0.41,  0.33],
                            [-0.39,  0.58],
                            [-0.37,  0.58],
                            [-0.37,  0.58],
                            [-0.35,  0.59],
                            [-0.35,  0.6 ],
                            [-0.35,  0.61],
                            [-0.34,  0.61],
                            [-0.34,  0.62],
                            [-0.28,  0.86],
                            [-0.28,  0.86],
                            [-0.26,  0.87],
                            [-0.26,  0.88],
                            [-0.26,  0.89],
                            [-0.25,  0.9 ],
                            [-0.25,  0.92],
                            [-0.18 , 1.13],
                            [-0.16 , 1.15],
                            [-0.16 , 1.16],
                            [-0.15 , 1.17],
                            [-0.14 , 1.19],
                            [-0.06 , 1.39],
                            [-0.04 , 1.39],
                            [-0.02 , 1.39],
                            [-0.   , 1.39],
                            [ 0.64 , 1.06],
                            [ 0.67 , 1.06],
                            [ 0.67 , 1.05],
                            [ 0.67 , 1.05],
                            [ 0.56,  0.84],
                            [ 0.56,  0.82],
                            [ 0.55,  0.8 ],
                            [ 0.55,  0.8 ],
                            [ 0.55,  0.78],
                            [ 0.55,  0.78],
                            [ 0.54,  0.58],
                            [ 0.52,  0.56],
                            [ 0.51,  0.55],
                            [ 0.51,  0.54],
                            [ 0.51,  0.53],
                            [ 0.52,  0.52],
                            [ 0.51,  0.52],
                            [ 0.52,  0.32],
                            [ 0.52,  0.32],
                            [ 0.51,  0.31],
                            [ 0.51,  0.3 ],
                            [ 0.5 ,  0.3 ],
                            [ 0.5 ,  0.29],
                            [ 0.49,  0.28]])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]


    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)



def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")



        
def DWA_Node(Node):

    def __init__(self):
        super().__init__('dynamic_window_approach_node')

        self.get_logger().info('Dynamic Window Node started!')
    
            
def main(args=None, gx=2.0, gy=2.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")


    rclpy.init(args=args)

    node = DWA_Node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi/2, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob
    fig, ax = plt.subplots()
    
    plt.ion()
    plt.show()
    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:


            plt.cla()
            # for stopping simulation with the esc key.
            
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            
            plt.plot(x[0], x[1], "xr")
    
            plt.plot(goal[0], goal[1], "xb")
            
            plt.plot(ob[:, 0], ob[:, 1], "ok")
  
            plot_robot(x[0], x[1], x[2], config)
         
            plot_arrow(x[0], x[1], x[2])
            
            plt.axis("equal")
            plt.grid(True)
            fig.canvas.draw()
            image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            ims.append(image)
            plt.pause(0.0001)
        
           

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        fig.canvas.draw()
        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        ims.append(image)
        
        plt.pause(0.0001)
    imageio.mimsave('animation.gif', ims, fps=10)

    plt.show()


if __name__ == '__main__':
    main(robot_type=RobotType.circle)
    # main(robot_type=RobotType.circle)