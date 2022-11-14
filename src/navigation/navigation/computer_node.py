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

    def __init__(self, name):
        super().__init__('computer_node')

        self.movement_vector = Vector3()
        self.speed = 0.5
        self.action = Twist()
        self.pub_action = self.create_publisher(Twist, 'cmd_vel', 10)

        self.sub_img = self.create_subscription(Image, 'robot_node/image', self.cb_image, 10)

        self.bridge = CvBridge()

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def cb_image(self, imgmsg):
        image = self.bridge.imgmsg_to_cv2(imgmsg, 'bgr8')

        cv2.imshow('image', image)
        cv2.waitKey(1)

    def on_press(self, key):

        action = Twist()

        if key == keyboard.Key.up or key == keyboard.Key.down or keyboard.Key.left or keyboard.Key.right or keyboard.Key.shift or keyboard.Key.ctrl or keyboard.Key.space:

            if key == keyboard.Key.up:
                self.movement_vector.x = self.speed

            elif key == keyboard.Key.down:
                self.movement_vector.x = -self.speed

            elif key == keyboard.Key.left:
                self.movement_vector.z = self.speed

            elif key == keyboard.Key.right:
                self.movement_vector.z = -self.speed

            elif key == keyboard.Key.shift:
                if self.speed <= 1:
                    self.speed += 0.1

            elif key == keyboard.Key.ctrl:
                if self.speed >= 0:
                    self.speed -= 0.1

            elif key == keyboard.Key.space:
                    self.speed = 0

            action.linear.x = self.movement_vector.x
            action.angular.z = self.movement_vector.z
            self.pub_action.publish(action)

    def on_release(self, key):

        action = Twist()

        if key == keyboard.Key.up:
            self.movement_vector.x = 0

        elif key == keyboard.Key.down:
            self.movement_vector.x = 0

        elif key == keyboard.Key.left:
            self.movement_vector.z = 0

        elif key == keyboard.Key.right:
            self.movement_vector.z = 0

        action.linear.x = self.movement_vector.x
        action.angular.z = self.movement_vector.z
        self.pub_action.publish(action)


def main(args=None):
    rclpy.init(args=args)

    node = Computer_Node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
