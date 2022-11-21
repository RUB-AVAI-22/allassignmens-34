import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult

import cv2
import numpy as np
from pynput import keyboard

from inputs import get_gamepad
from inputs import UnpluggedError


class Computer_Node(Node):

    def __init__(self):
        super().__init__('computer_node')

        self.maxTransVelocity = 0.26 #in m/s
        self.maxRotVelocity = 1.82 #in rad/s
        self.speed = 0.1
        self.acceleration = 0.01
        
        self.currentMovement = Vector3() #x = translational, z = rotational
        self.desiredMovement = Vector3() #x = translational, z = rotational
        
        self.pub_action = self.create_publisher(Twist, 'cmd_vel', 10)

        self.sub_img = self.create_subscription(Image, 'robot_node/image', self.cb_image, 10)

        self.bridge = CvBridge()

        self.updateFrequency = 20
        self.velocityUpdateTimer = self.create_timer(1 / self.updateFrequency, self.updateVelocity)
        
        self.controller = True
        self.controllerInputTimer = self.create_timer(1/self.updateFrequency, self.updateControllerInput)
        
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def cb_image(self, imgmsg):
        image = self.bridge.imgmsg_to_cv2(imgmsg, 'bgr8')

        cv2.imshow('image', image)
        cv2.waitKey(1)

    def on_press(self, key):
        speedChange = False

        if key == keyboard.Key.up or key == keyboard.Key.down or keyboard.Key.left or keyboard.Key.right or keyboard.Key.shift or keyboard.Key.ctrl or keyboard.Key.space:

            if key == keyboard.Key.up:
                self.desiredMovement.x = self.speed

            elif key == keyboard.Key.down:
                self.desiredMovement.x = -self.speed

            elif key == keyboard.Key.left:
                self.desiredMovement.z = 7*self.speed*np.sign(self.desiredMovement.x)

            elif key == keyboard.Key.right:
                self.desiredMovement.z = -7*self.speed*np.sign(self.desiredMovement.x)

            elif key == keyboard.Key.shift:
                if self.speed <= self.maxTransVelocity:
                    self.speed += 0.01
                    speedChange = True

            elif key == keyboard.Key.ctrl:
                if self.speed >= 0:
                    self.speed -= 0.01
                    speedChange = True

            elif key == keyboard.Key.space:
                    self.speed = 0.0
                    speedChange = True

            if speedChange:
                print(f"New speed: {self.speed}")
            

    def on_release(self, key):

        if key == keyboard.Key.up:
            self.desiredMovement.x = 0.0

        elif key == keyboard.Key.down:
            self.desiredMovement.x = 0.0

        elif key == keyboard.Key.left:
            self.desiredMovement.z = 0.0

        elif key == keyboard.Key.right:
            self.desiredMovement.z  = 0.0


    def updateVelocity(self):
        action = Twist()
        
        distanceToDesiredVelocity = self.desiredMovement - self.currentMovement
        
        if distanceToDesiredVelocity.x > 0 or distanceToDesiredVelocity.z > 0:
            self.currentMovement.x += min(distanceToDesiredVelocity.x, self.acceleration)
            self.currentMovement.z += min(distanceToDesiredVelocity.x, 7*self.acceleration)
            
            
            action.linear.x = self.currentMovement.x
            action.angular.z = self.currentMovement.z
            self.pub_action.publish(action)
            
            print(f"Current velocity: {self.currentMovement.x} m/s, {self.currentMovement.z} rad/s")
            print(f"Desired velocity: {self.desiredMovement.x} m/s, {self.desiredMovement.z} rad/s\n")

    def updateControllerInput(self):
        if self.controller:
            try:
                events = get_gamepad()
                for event in events:
                    if event.code == "HX":
                        self.desiredMovement.x = (event.state/32768)*self.maxTransVelocity
                    elif event.code == "HY":
                        self.desiredMovement.z = -(event.state/32768)*self.maxRotVelocity*np.sign(self.desiredMovement.x)
            except UnpluggedError:
                print("No gamepad plugged in. Gamepad support will be disabled from now on!")
                self.controller = False

def main(args=None):
    rclpy.init(args=args)

    node = Computer_Node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
