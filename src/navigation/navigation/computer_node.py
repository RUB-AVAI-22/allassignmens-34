import rclpy
from rclpy.node import Node

import cv2
import time
import math
from decimal import *

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg._laser_scan import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from pynput import keyboard
import pygame
pygame.init()
class Computer_Node(Node):

    def __init__(self):
        super().__init__('computer_node')

        self.ox = []
        self.oy = []
        self.x_turtle = 0.0
        self.y_turtle = 0.0
        self.theta_turtle = 0.0

        self.pos_stp_sec = 0
        self.pos_stp_nanosec = 0

        self.lidar_stp_sec = 0
        self.lidar_stp_nanosec = 0

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
        
        self.pub_action = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.cluster_points_in_fov,
                                                                qos_profile_sensor_data)
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.cb_image, 10)
        self.lidar_sensor_subscriber = self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.bridge = CvBridge()

        self.updateFrequency = 5
        self.acceleration = 0.25/self.updateFrequency
        self.velocityUpdateTimer = self.create_timer(1 / self.updateFrequency, self.updateVelocity)
        
        self.gamePadvelocityUpdateTimer = self.create_timer(1 / self.updateFrequency, self.updateGamePad)

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def odom(self, odommsg):
        #self.twisted = odommsg.pose.pose
        if odommsg.pose.pose.orientation.x > 0:
           self.theta_turtle = round(math.acos(odommsg.pose.pose.orientation.w)*180/math.pi*2,1)
        elif odommsg.pose.pose.orientation.x < 0:
            self.theta_turtle = round(-math.acos(odommsg.pose.pose.orientation.w)*180/math.pi*2,1)
        self.x_turtle = odommsg.pose.pose.position.x
        self.y_turtle = odommsg.pose.pose.position.y

        self.pos_stp_sec = odommsg.header.stamp.sec
        self.pos_stp_nanosec = odommsg.header.stamp.nanosec

    def cluster_points_in_fov(self, lidar):
        #reads the lidar data and clusters the points in the camera fov
        #returns an array of points as (middle_point in degree, distance)
        #(31, 1.5) means right in the center of the camera there is a object 1,5m away

        
        laser_scan_list = lidar.ranges

        self.lidar_stp_sec = lidar.header.stamp.sec
        self.lidar_stp_nanosec = lidar.header.stamp.nanosec

        laser_scan_list.reverse()

        
        xy_resolution = 0.002 
        angles = []
        distances = []
        plt.ion()
        for current_degree, distance in enumerate(laser_scan_list):
            if distance != float("inf"):
                angles.append((current_degree + self.theta_turtle) * math.pi / 180)
                distances.append(float(distance))
          
        ox = np.sin(angles) * distances - np.ones(len(distances)) * self.y_turtle
        oy = np.cos(angles) * distances + np.ones(len(angles)) * self.x_turtle

        if abs(self.lidar_stp_sec - self.pos_stp_sec) == 0 and abs(self.lidar_stp_nanosec - self.pos_stp_nanosec) < 13000000:
            self.ox.extend(ox.tolist())
            self.oy.extend(oy.tolist())
        plt.scatter(self.ox, self.oy, s= 2)
        plt.xlim(xmin = -10)
        plt.xlim(xmax = 10)
        plt.ylim(ymin = -10)
        plt.ylim(ymax = 10)
        plt.pause(0.001)
        plt.ioff()
        plt.clf()     

    def cb_image(self, imgmsg):
        image = self.bridge.imgmsg_to_cv2(imgmsg, 'bgr8')

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

    node = Computer_Node()
    rclpy.spin(node)
    plt.show()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
