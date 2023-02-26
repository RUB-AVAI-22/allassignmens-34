import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import cv2
import time
import math
from decimal import *

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg._laser_scan import LaserScan

import message_filters

import numpy as np
import matplotlib.pyplot as plt

from pynput import keyboard
import scipy.signal as signal

import _thread

import math
from enum import Enum

import matplotlib.pyplot as plt


EXTEND_AREA = 1.0

import pygame
pygame.init()

show_animation = True





class RobotType(Enum):
    circle = 1
    rectangle = 0
    
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
        self.dt = 0.05 # [s] Time tick for motion prediction
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 0.1
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 0.4
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.306  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]


    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()



class Odom_Node(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.x_turtle = 0.0
        self.y_turtle = 0.0
        self.yaw = 0
        self.group = ReentrantCallbackGroup()
        self.odom_turtle = self.create_subscription(Odometry, '/odom', self.odom, 50, callback_group=self.group)

    def odom(self, odommsg):

        self.x_turtle = odommsg.pose.pose.position.x
        self.y_turtle = odommsg.pose.pose.position.y
        self.yaw = odommsg.pose.pose.orientation.w


class Computer_Node(Node):

    def __init__(self):
        super().__init__('computer_node')

        self.node: Odom_Node = None

        self.ox = []
        self.oy = []
        self.x_turtle = 0.0
        self.y_turtle = 0.0
        self.yaw = 0
        self.x_turtle_old = 0.0
        self.y_turtle_old = 0.0
        self.yaw_old = math.pi / 2.0

        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.x = np.array([self.x_turtle, self.y_turtle , math.pi / 2.0, 0.0, 0.0])
        # goal position [x(m), y(m)]
        self.goal = np.array([10.0, 10.0])

        config.robot_type = RobotType.circle
        self.trajectory = np.array(self.x)

        self.left_right = []
        self.theta_turtle = 0.0
        

        self.pos_stp_sec = 0
        self.pos_stp_nanosec = 0

        self.lidar_stp_sec = 0
        self.lidar_stp_nanosec = 0

        self.crone_position = []
        self.turtle_track_x = []
        self.turtle_track_y = []

        self.maxTransVelocity = 0.26 #in m/s
        self.maxRotVelocity = 1.82 #in rad/s

        self.speed = 0.8
        self.counter = 0
        self.twisted = 0
        self.joysticks = {}
        self.Unlock = False
        self.unlockCounter = 3
        self.Unlockchecker = False
        self.useGamePad = False

        self.currentMovement = Vector3() #x = translational, z = rotational
        self.desiredMovement = Vector3() #x = translational, z = rotational
        self.group = ReentrantCallbackGroup()
        self.pub_action = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.autopilotTimer = self.create_timer(0.001, self.autopilot, callback_group=self.group)
        #self.lidar_sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.cluster_points_in_fov,
        #                                                        qos_profile_sensor_data)
        
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.cb_image, qos_profile=qos_profile_sensor_data,callback_group=self.group)

        self.lidar_sensor_subscriber = message_filters.Subscriber(self, LaserScan, 'scan', qos_profile=qos_profile_sensor_data)
        self.odom_subscriber = message_filters.Subscriber(self, Odometry, 'odom')
        self.sub_img = message_filters.Subscriber(self, Image, 'camera/image_raw')

        ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sensor_subscriber, self.odom_subscriber], 50, 0.1,False)
        ts.registerCallback(self.callback)

    

        self.updateFrequency = 5
        self.acceleration = 0.25/self.updateFrequency


        self.velocityUpdateTimer = self.create_timer(1 / self.updateFrequency, self.updateVelocity)
        
        
        #self.gamePadvelocityUpdateTimer = self.create_timer(1 / self.updateFrequency, self.updateGamePad)

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        plt.ion()
        plt.show()

    



    def callback(self, lidar, odommsg):


        """
            get odom info
        """
        if odommsg.pose.pose.orientation.x > 0:
           theta_turtle = round(math.acos(odommsg.pose.pose.orientation.w)*180/math.pi*2,1)
        elif odommsg.pose.pose.orientation.x < 0:
            theta_turtle = round(-math.acos(odommsg.pose.pose.orientation.w)*180/math.pi*2,1)
        x_turtle = odommsg.pose.pose.position.x
        y_turtle = odommsg.pose.pose.position.y
        yaw = odommsg.pose.pose.orientation.w

        """
            get lidar info
        """

        laser_scan_list = lidar.ranges
        laser_scan_list.reverse()

        
        
        angles = []
        distances = []
        left_right =[]
        
        for current_degree, distance in enumerate(laser_scan_list):
            #if 30 <= current_degree <= 90 or 330 >= current_degree >= 270:
                if distance != float("inf") and distance < 1 :

                    angles.append((current_degree + theta_turtle) * math.pi / 180)
                    distances.append(float(distance))
                    if current_degree <179:
                        left_right.append("r")
                    else:
                        left_right.append("b")


        
        ox = np.round(np.sin(angles) * distances - np.ones(len(distances)) * y_turtle,2) # 
        oy = np.round(np.cos(angles) * distances + np.ones(len(distances)) * x_turtle,2) #  

        if len(self.turtle_track_x) == 0 or abs(self.turtle_track_x[-1] - self.x_turtle) > 0.01 or abs(self.turtle_track_y[-1] - y_turtle) > 0.01:
            self.turtle_track_x.append(round(x_turtle,2))
            self.turtle_track_y.append(round(-y_turtle,2))

        self.ox.extend(ox.tolist())
        self.oy.extend(oy.tolist())
        self.left_right.extend(left_right)
        self.crone_position  = np.array((signal.medfilt(volume=self.ox, kernel_size=5),signal.medfilt(volume=self.oy, kernel_size=5))).T

        #self.autopilot()
        
        # if len(self.crone_position)  == 0:
        #     return 0
        # plt.scatter(self.crone_position[:, 0], self.crone_position[:, 1], s= 0.5)
        # plt.xlim(xmin = -5)
        # plt.xlim(xmax = 5)
        # plt.ylim(ymin = -5)
        # plt.ylim(ymax = 5)
        # plt.grid(True)
        # plt.pause(0.001)
        # plt.ioff()
        # plt.clf()  

        

    def cluster_points_in_fov(self, lidar):
        #reads the lidar data and clusters the points in the camera fov
        #returns an array of points as (middle_point in degree, distance)
        #(31, 1.5) means right in the center of the camera there is a object 1,5m away

        
        laser_scan_list = lidar.ranges

        self.lidar_stp_sec = lidar.header.stamp.sec
        self.lidar_stp_nanosec = lidar.header.stamp.nanosec

        laser_scan_list.reverse()

        
        
        angles = []
        distances = []
        left_right =[]
        
        for current_degree, distance in enumerate(laser_scan_list):
            #if 30 <= current_degree <= 90 or 330 >= current_degree >= 270:
                if distance != float("inf") and distance < 2 :

                    angles.append((current_degree + self.theta_turtle) * math.pi / 180)
                    distances.append(float(distance))
                    if current_degree <179:
                        left_right.append("r")
                    else:
                        left_right.append("b")


        
        ox = np.round(np.sin(angles) * distances - np.ones(len(distances)) * self.y_turtle,2) # 
        oy = np.round(np.cos(angles) * distances + np.ones(len(distances)) * self.x_turtle,2) #  

        if len(self.turtle_track_x) == 0 or abs(self.turtle_track_x[-1] - self.x_turtle) > 0.01 or abs(self.turtle_track_y[-1] - self.y_turtle) > 0.01:
            self.turtle_track_x.append(round(self.x_turtle,2))
            self.turtle_track_y.append(round(-self.y_turtle,2))

        #if len(self.turtle_track_x) > 0 and  abs(self.turtle_track_x[-1] - self.x_turtle) > 0.01 or len(self.turtle_track_y) > 0 and  abs(self.turtle_track_y[-1] - self.y_turtle) > 0.01:
        #    self.turtle_track_x.append(self.x_turtle)
        #    self.turtle_track_y.append(-self.y_turtle)
            
        if abs(self.lidar_stp_sec - self.pos_stp_sec) == 0 and abs(self.lidar_stp_nanosec - self.pos_stp_nanosec) < 13000000:
            self.ox.extend(ox.tolist())
            self.oy.extend(oy.tolist())
            self.left_right.extend(left_right)
            self.crone_position  = np.array((signal.medfilt(volume=self.ox, kernel_size=5),signal.medfilt(volume=self.oy, kernel_size=5))).T

            
           
            
            #np.array([np.array(self.ox), np.array(self.oy)])

            #svm = SVC(kernel = 'rbf', random_state = 0, gamma = 0.2, C = 0.2)
            #svm.fit(self.crone_position)
            # 可视化分类结果


            #y_pred = DBSCAN(eps = 0.05, min_samples=5).fit_predict(self.crone_position)
            
            # plt.scatter(self.crone_position[:, 0], self.crone_position[:, 1], c= self.left_right,  s= 0.5)
            # plt.plot(self.turtle_track_y, self.turtle_track_x)
            # #plt.scatter(X[:, 0], X[:, 1],  s= 2)
             
            
            # xy_resolution = 0.02
            # self.ox.reverse()
            # #self.oy.reverse()
            # occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = \
            # self.generate_ray_casting_grid_map(self.ox, self.oy, xy_resolution, True)
            # xy_res = np.array(occupancy_map).shape
            # plt.figure(1, figsize=(10, 4))
            # plt.subplot(122)
            # plt.imshow(occupancy_map, cmap="PiYG_r")
            # # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
            # plt.clim(-0.4, 1.4)
            # plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
            # plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
            # plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
            # plt.colorbar()
            # plt.subplot(121)
            # plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
            # plt.axis("equal")
            # plt.plot(0.0, 0.0, "ob")
            # plt.gca().set_aspect("equal", "box")
            # bottom, top = plt.ylim()  # return the current y-lim
            # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
            # plt.grid(True)
            

    def bresenham(self, start, end):
        """
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        Bresenham's Line Algorithm
        Produces a np.array from start and end (original from roguebasin.com)
        >>> points1 = bresenham((4, 4), (6, 10))
        >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """
        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        points = np.array(points)
        return points


    def calc_grid_map_config(self, ox, oy, xy_resolution):
        """
        Calculates the size, and the maximum distances according to the the
        measurement center
        """
        min_x = round(min(ox) - EXTEND_AREA / 2.0)
        min_y = round(min(oy) - EXTEND_AREA / 2.0)
        max_x = round(max(ox) + EXTEND_AREA / 2.0)
        max_y = round(max(oy) + EXTEND_AREA / 2.0)
        xw = int(round((max_x - min_x) / xy_resolution))
        yw = int(round((max_y - min_y) / xy_resolution))
        print("The grid map is ", xw, "x", yw, ".")
        return min_x, min_y, max_x, max_y, xw, yw


    def atan_zero_to_twopi(y, x):
        angle = math.atan2(y, x)
        if angle < 0.0:
            angle += math.pi * 2.0
        return angle


    def init_flood_fill(self, center_point, obstacle_points, xy_points, min_coord,
                        xy_resolution):
        """
        center_point: center point
        obstacle_points: detected obstacles points (x,y)
        xy_points: (x,y) point pairs
        """
        center_x, center_y = center_point
        prev_ix, prev_iy = center_x - 1, center_y
        ox, oy = obstacle_points
        xw, yw = xy_points
        min_x, min_y = min_coord
        occupancy_map = (np.ones((xw, yw))) * 0.5
        for (x, y) in zip(ox, oy):
            # x coordinate of the the occupied area
            ix = int(round((x - min_x) / xy_resolution))
            # y coordinate of the the occupied area
            iy = int(round((y - min_y) / xy_resolution))
            free_area = self.bresenham((prev_ix, prev_iy), (ix, iy))
            for fa in free_area:
                occupancy_map[fa[0]][fa[1]] = 0  # free area 0.0
            prev_ix = ix
            prev_iy = iy
        return occupancy_map


    def flood_fill(self, center_point, occupancy_map):
        """
        center_point: starting point (x,y) of fill
        occupancy_map: occupancy map generated from Bresenham ray-tracing
        """
        # Fill empty areas with queue method
        sx, sy = occupancy_map.shape
        fringe = self.deque()
        fringe.appendleft(self.center_point)
        while fringe:
            n = fringe.pop()
            nx, ny = n
            # West
            if nx > 0:
                if occupancy_map[nx - 1, ny] == 0.5:
                    occupancy_map[nx - 1, ny] = 0.0
                    fringe.appendleft((nx - 1, ny))
            # East
            if nx < sx - 1:
                if occupancy_map[nx + 1, ny] == 0.5:
                    occupancy_map[nx + 1, ny] = 0.0
                    fringe.appendleft((nx + 1, ny))
            # North
            if ny > 0:
                if occupancy_map[nx, ny - 1] == 0.5:
                    occupancy_map[nx, ny - 1] = 0.0
                    fringe.appendleft((nx, ny - 1))
            # South
            if ny < sy - 1:
                if occupancy_map[nx, ny + 1] == 0.5:
                    occupancy_map[nx, ny + 1] = 0.0
                    fringe.appendleft((nx, ny + 1))


    def generate_ray_casting_grid_map(self, ox, oy, xy_resolution, breshen=True):
        """
        The breshen boolean tells if it's computed with bresenham ray casting
        (True) or with flood fill (False)
        """
        min_x, min_y, max_x, max_y, x_w, y_w = self.calc_grid_map_config(
            ox, oy, xy_resolution)
        # default 0.5 -- [[0.5 for i in range(y_w)] for i in range(x_w)]
        occupancy_map = np.ones((x_w, y_w)) / 2
        center_x = int(
            round(-min_x / xy_resolution))  # center x coordinate of the grid map
        center_y = int(
            round(-min_y / xy_resolution))  # center y coordinate of the grid map
        # occupancy grid computed with bresenham ray casting
        if breshen:
            for (x, y) in zip(ox, oy):
                # x coordinate of the the occupied area
                ix = int(round((x - min_x) / xy_resolution))
                # y coordinate of the the occupied area
                iy = int(round((y - min_y) / xy_resolution))
                laser_beams = self.bresenham((center_x, center_y), (
                    ix, iy))  # line form the lidar to the occupied point
                for laser_beam in laser_beams:
                    occupancy_map[laser_beam[0]][
                        laser_beam[1]] = 0.0  # free area 0.0
                occupancy_map[ix][iy] = 1.0  # occupied area 1.0
                occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
                occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
                occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
        # occupancy grid computed with with flood fill
        else:
            occupancy_map = self.init_flood_fill((center_x, center_y), (ox, oy),
                                            (x_w, y_w),
                                            (min_x, min_y), xy_resolution)
            self.flood_fill((center_x, center_y), occupancy_map)
            occupancy_map = np.array(occupancy_map, dtype=float)
            for (x, y) in zip(ox, oy):
                ix = int(round((x - min_x) / xy_resolution))
                iy = int(round((y - min_y) / xy_resolution))
                occupancy_map[ix][iy] = 1.0  # occupied area 1.0
                occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
                occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
                occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
        return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution
    

    def autopilot(self):

        if len(self.crone_position) == 0:
            return 0

        ob = self.crone_position
        self.x[0]=self.x_turtle_old
        self.x[1]=self.y_turtle_old
        self.x[2]=self.yaw_old
        u, predicted_trajectory = self.dwa_control(self.x, config, self.goal, ob)
        self.x = self.robot_motion(self.x, u, config.dt)  # simulate robot
        
        self.trajectory = np.vstack((self.trajectory, self.x))  # store state history
    

        # if show_animation:
        #     plt.cla()
        #     # for stopping simulation with the esc key.
        #     plt.gcf().canvas.mpl_connect(
        #         'key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None])
        #     plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
        #     plt.plot(self.x[0], self.x[1], "xr")
        #     plt.plot(self.goal[0], self.goal[1], "xb")
        #     plt.plot(ob[:, 0], ob[:, 1], "ok")
        #     self.plot_robot(self.x[0], self.x[1], self.x[2], config)
        #     self.plot_arrow(self.x[0], self.x[1], self.x[2])
        #     plt.axis("equal")
        #     plt.grid(True)
        #     plt.pause(0.001)
        #     plt.ioff()
        #     plt.clf()   

        # check reaching goal
        # dist_to_goal = math.hypot(self.x[0] - self.goal[0], self.x[1] - self.goal[1])
        # if dist_to_goal <= config.robot_radius:
        #     print("Goal!!")
        
    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

    def robot_motion(self, x, u, dt):
        """
        motion model
        """
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        

        x[3] = u[0] #v(m/s)
        x[4] = u[1] #omega(rad/s)

        action = Twist()

        rotation_t1 = time.perf_counter()

        before =self.node.x_turtle
        while True:
            action.linear.x = 0.0
            action.angular.z = u[1]
            self.pub_action.publish(action)
            rotation_t2 = time.perf_counter()
            
            if rotation_t2 - rotation_t1 > dt:
                action.linear.x = 0.0
                action.angular.z = 0.0
                self.pub_action.publish(action)
                break
        time.sleep(0.3)
        after = self.node.x_turtle
        print("Diff:", after-before)
        #print("new", Odom_Node.yaw)

        t1 = time.perf_counter()

        while True:
            action.linear.x = u[0]
            action.angular.z = 0.0
            self.pub_action.publish(action)
            t2 = time.perf_counter()
            if abs(u[0] * math.cos(x[2]) * dt - abs(self.x_turtle-self.x_turtle_old)) < 0.1:
                action.linear.x = 0.0
                action.angular.z = 0.0
                self.pub_action.publish
                break
        
        # self.x_turtle_old = self.x_turtle
        # self.y_turtle_old = self.y_turtle
        self.yaw_old = self.yaw
        # print("old",self.x_turtle_old)
        # print("new",self.x_turtle)

        return x
        
        


    def calc_dynamic_window(self, x, config):
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


    def predict_trajectory(self, x_init, v, y, config):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= config.predict_time:
            x = self.motion(x, [v, y], config.dt)
            trajectory = np.vstack((trajectory, x))
            time += config.dt

        return trajectory


    def calc_control_and_trajectory(self, x, dw, config, goal, ob):
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

                trajectory = self.predict_trajectory(x_init, v, y, config)
                # calc cost
                to_goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
                ob_cost = config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob, config)

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


    def calc_obstacle_cost(self, trajectory, ob, config):
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


    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost


    def plot_arrow(self, x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        plt.plot(x, y)


    def plot_robot(self, x, y, yaw, config):  # pragma: no cover
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


    def dwa_control(self, x, config, goal, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x, config)

        u, trajectory = self.calc_control_and_trajectory(x, dw, config, goal, ob)

        return u, trajectory


    def cb_image(self, imgmsg):

        image = CvBridge().imgmsg_to_cv2(imgmsg, 'bgr8')

        cv2.imshow('image', image)
        cv2.waitKey(1)

    def on_press(self, key):
        speedChange = False

        if hasattr(key, 'char'):#if key != keyboard.Key.up and key != keyboard.Key.down and key != keyboard.Key.left and key != keyboard.Key.right and key != keyboard.Key.shift and key != keyboard.Key.ctrl and key != keyboard.Key.space:
            if key.char == 'g':
                self.useGamePad = True
                print('\ncontrol switch to gamepad')

        if not self.useGamePad:
            if key == keyboard.Key.up or keyboard.Key.down or keyboard.Key.left or keyboard.Key.right or keyboard.Key.shift or keyboard.Key.ctrl or keyboard.Key.space:

                if key == keyboard.Key.up:
                    self.desiredMovement.x = self.speed*self.maxTransVelocity

                elif key == keyboard.Key.down:
                    self.desiredMovement.x = -self.speed*self.maxTransVelocity

                elif key == keyboard.Key.left:
                    self.desiredMovement.z = self.speed*self.maxRotVelocity# * (2*(self.desiredMovement.x > 0)-1)
                    

                elif key == keyboard.Key.right:              
                    self.desiredMovement.z = -self.speed*self.maxRotVelocity# * (2*(self.desiredMovement.x > 0)-1)


                elif key == keyboard.Key.shift:
                    if self.speed < 1:
                        self.speed =  round(self.speed + 0.1,1)
                        speedChange = True

                elif key == keyboard.Key.ctrl:
                    if self.speed > 0:
                        self.speed =  round(self.speed - 0.1,1)
                        speedChange = True

                elif key == keyboard.Key.space:
                    if not self.speed == 0:
                        self.speed = 0.0
                        speedChange = True

                if speedChange:
                    print(f"\nspeed multiplier : {self.speed}")
            

    def on_release(self, key):

        if key == keyboard.Key.up:
            self.desiredMovement.x = 0.0

        elif key == keyboard.Key.down:
            self.desiredMovement.x = 0.0

        elif key == keyboard.Key.left:
            self.desiredMovement.z = 0.0
            
        elif key == keyboard.Key.right:
            self.desiredMovement.z = 0.0
            self.counter = 0

    def updateVelocity(self):
        action = Twist()
        
        distanceToDesiredVelocity = Vector3()
        distanceToDesiredVelocity.x = self.desiredMovement.x - self.currentMovement.x
        distanceToDesiredVelocity.y = self.desiredMovement.y - self.currentMovement.y
        distanceToDesiredVelocity.z = self.desiredMovement.z - self.currentMovement.z

        if abs(distanceToDesiredVelocity.x) >= self.acceleration:
            self.currentMovement.x += self.acceleration * (2*(distanceToDesiredVelocity.x > 0)-1)
        else:
            self.currentMovement.x += distanceToDesiredVelocity.x

        if abs(distanceToDesiredVelocity.z) >= 20*self.acceleration:
            self.currentMovement.z += 20*self.acceleration * (2*(distanceToDesiredVelocity.z > 0)-1)
        else:
            self.currentMovement.z += distanceToDesiredVelocity.z

        action.linear.x = round(self.desiredMovement.x,2)
        action.angular.z = round(self.desiredMovement.z,2)
        
        self.pub_action.publish(action)

        
        #time.sleep(abs(round(-90 * (math.pi / 180),2))/1.82)
               
        if not self.useGamePad:
            print("\r                                                                     ",end='')
            print("\r\rCurrent velocity: {:.3} m/s, {:.3} rad/s  (Keyboard)\r".format(action.linear.x,action.angular.z),end='')
        #print(f"\rDesired velocity: {self.desiredMovement.x} m/s, {self.desiredMovement.z} rad/s",end='')

    def updateGamePad(self):

        if self.useGamePad:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True  # Flag that we are done so we exit this loop.

                #if event.type == pygame.JOYBUTTONDOWN:
                #    print("Joystick button pressed.")

                #if event.type == pygame.JOYBUTTONUP:
                #    print("Joystick button released.")

                # Handle hotplugging
                if event.type == pygame.JOYDEVICEADDED:
                    # This event will be generated when the program starts for every
                    # joystick, filling up the list without needing to create them manually.
                    joy = pygame.joystick.Joystick(event.device_index)
                    self.joysticks[joy.get_instance_id()] = joy
                    print(f"Joystick {joy.get_instance_id()} connencted")

                if event.type == pygame.JOYDEVICEREMOVED:
                    del self.joysticks[event.instance_id]
                    print(f"Joystick {event.instance_id} disconnected")
            
            for joystick in self.joysticks.values():
                action = Twist()
                axis_x1 = joystick.get_axis(0)
                axis_y1 = joystick.get_axis(1)

                axis_x2 = joystick.get_axis(3)
                axis_y2 = joystick.get_axis(4)
                if 0.5 < axis_x1 < 0.9 and 0.5 < axis_y1 < 0.9 and -0.9 < axis_x2 < -0.5 and 0.5 < axis_y2 < 0.9:
                    if self.unlockCounter > 0:
                        self.unlockCounter -= 1
                        print(f'keep press {self.unlockCounter}s')
                        time.sleep(1)
                        break
                    elif self.Unlock == False:
                        print('ggamepad is unlocked!')
                        joystick.rumble(0, 0.5, 1000)
                        self.Unlock = True
                
                #else:
                #    print('unlock failed please try again!')
                #    self.Unlockchecker == False

                if joystick.get_button(0) == 1:

                    if self.Unlock == True:
                        
                        self.Unlock = False
                        self.Unlockchecker = False
                        self.unlockCounter = 3
                        print('\ngamepad is locked')
                        joystick.rumble(0, 1, 500)


                if joystick.get_button(3) == 1:
                    self.useGamePad = False
                    self.unlgockCounter = 3
                    print('gcontrol switch to keyboard')

                if self.Unlock:
                    if abs(axis_x1) < 0.05:
                        axis_x1 = 0
                    if abs(axis_y1) < 0.05:
                        axis_y1 = 0
                    action.linear.x = round(self.maxTransVelocity * -axis_y1,2)
                    action.angular.z = round(self.maxRotVelocity * -(axis_x1*axis_x1*axis_x1),2)
                    self.pub_action.publish(action)
                    print("\r                                                                     ",end='')
                    print(f"\rCurrent velocity: {action.linear.x} m/s, {action.angular.z} rad/s  (Gamepad)\r", end='')



def main(args=None):
    rclpy.init(args=args)
    try:
        node = Computer_Node()
        odom_node = Odom_Node()
        node.node = odom_node
        # MultiThreadedExecutor executes callbacks with a thread pool. If num_threads is not
        # specified then num_threads will be multiprocessing.cpu_count() if it is implemented.
        # Otherwise it will use a single thread. This executor will allow callbacks to happen in
        # parallel, however the MutuallyExclusiveCallbackGroup in DoubleTalker will only allow its
        # callbacks to be executed one at a time. The callbacks in Listener are free to execute in
        # parallel to the ones in DoubleTalker however.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.add_node(odom_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()




if __name__ == '__main__':
    main()
