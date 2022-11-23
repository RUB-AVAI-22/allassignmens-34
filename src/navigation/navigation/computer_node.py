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


class Computer_Node(Node):

    def __init__(self):
        super().__init__('computer_node')

        self.maxTransVelocity = 0.26 #in m/s
        self.maxRotVelocity = 1.82 #in rad/s
        self.speed = 0.1
        
        self.currentMovement = Vector3() #x = translational, z = rotational
        self.desiredMovement = Vector3() #x = translational, z = rotational
        
        self.pub_action = self.create_publisher(Twist, 'cmd_vel', 10)

        self.sub_img = self.create_subscription(Image, 'robot_node/image', self.cb_image, 10)

        self.bridge = CvBridge()

        self.updateFrequency = 20
        self.acceleration = 0.1/self.updateFrequency
        self.velocityUpdateTimer = self.create_timer(1 / self.updateFrequency, self.updateVelocity)

        
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
                self.desiredMovement.x = self.speed*self.maxTransVelocity

            elif key == keyboard.Key.down:
                self.desiredMovement.x = -self.speed*self.maxTransVelocity

            elif key == keyboard.Key.left:
                if self.desiredMovement.x >= 0:
                    self.desiredMovement.z = 7*self.speed*self.maxRotVelocity
                else:
                    self.desiredMovement.z = -7*self.speed*self.maxRotVelocity

            elif key == keyboard.Key.right:
                if self.desiredMovement.x >= 0:
                    self.desiredMovement.z = -7*self.speed*self.maxRotVelocity
                else:
                    self.desiredMovement.z = 7*self.speed*self.maxRotVelocity

            elif key == keyboard.Key.shift:
                if self.speed < 1:
                    self.speed += 0.01
                    speedChange = True

            elif key == keyboard.Key.ctrl:
                if self.speed > 0:
                    self.speed -= 0.01
                    speedChange = True

            elif key == keyboard.Key.space:
                if not self.speed == 0:
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
            self.desiredMovement.z = 0.0


    def updateVelocity(self):
        action = Twist()
        
        distanceToDesiredVelocity = Vector3()
        distanceToDesiredVelocity.x = self.desiredMovement.x - self.currentMovement.x
        distanceToDesiredVelocity.y = self.desiredMovement.y - self.currentMovement.y
        distanceToDesiredVelocity.z = self.desiredMovement.z - self.currentMovement.z

        if abs(distanceToDesiredVelocity.x) >= self.acceleration:
            self.currentMovement.x += self.acceleration*(2*(distanceToDesiredVelocity.x>0)-1)
        else:
            self.currentMovement.x += distanceToDesiredVelocity.x

        if abs(distanceToDesiredVelocity.z) >= 14*self.acceleration:
            self.currentMovement.z += 14*self.acceleration*(2*(distanceToDesiredVelocity.z>0)-1)
        else:
            self.currentMovement.z += distanceToDesiredVelocity.z

        action.linear.x = self.currentMovement.x
        action.angular.z = self.currentMovement.z
        self.pub_action.publish(action)

        print(f"Current velocity: {self.currentMovement.x} m/s, {self.currentMovement.z} rad/s")
        print(f"Desired velocity: {self.desiredMovement.x} m/s, {self.desiredMovement.z} rad/s\n")

def main(args=None):
    rclpy.init(args=args)

    node = Computer_Node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
