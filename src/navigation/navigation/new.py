import rclpy
from rclpy.node import Node
from numpy import array
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from rcl_interfaces.msg import SetParametersResult

import cv2
import numpy as np
from pynput import keyboard


class Computer_Node(Node):

    def __init__(self, name):
        super().__init__('computer_node')




        self.pub_action = self.create_publisher(TwistStamped, '/cmd_vel_out', 10)
        self.pub_action2 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_img = self.create_subscription(Image, 'robot_node/image', self.cb_image, 10)

        self.bridge = CvBridge()

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def cb_image(self, imgmsg):
        image = self.bridge.imgmsg_to_cv2(imgmsg, 'bgr8')

        cv2.imshow('image', image)
        cv2.waitKey(1)

    def on_press(self, key):
        self.action2 = Twist()
        self.action = TwistStamped()
        #self.action.header.frame_id =
        print('some keyboard input detected')
        if key == keyboard.Key.up or key == keyboard.Key.down or keyboard.Key.left or keyboard.Key.right:
            if key == keyboard.Key.up:
                print('up key pressed')
                vector = geometry_msgs.msg.Vector3()
                vector.x = 0.04
                vector.y = 0.02
                self.action2.linear = vector
                self.action.twist.linear.x = 0.04
                self.action.twist.angular.x = 0.0
            elif key == keyboard.Key.down:
                self.action.twist.linear.x = 0.03
                self.action.twist.angular.x = 0.0
            elif key == keyboard.Key.left:
                self.action.twist.linear.x = 0.2
                self.action.twist.angular.x = 1.0
            elif key == keyboard.Key.right:
                self.action.twist.linear.x = 0.2
                self.action.twist.angular.x = -1.0

            self.action.header.stamp = self.get_clock().now().to_msg()
            self.pub_action.publish(self.action)
            self.pub_action2.publish(self.action2)

    def on_release(self, key):
        if key == keyboard.Key.up or key == keyboard.Key.down or keyboard.Key.left or keyboard.Key.right:
            #self.action.twist.linear.x = 0.0
            #self.action.twist.angular.x = 0.0
            #self.action.header.stamp = self.get_clock().now().to_msg()
            #self.pub_action.publish(self.action)
            print()

def main(args=None):
    rclpy.init(args=args)

    node = Computer_Node('control')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()