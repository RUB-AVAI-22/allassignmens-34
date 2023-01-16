import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import _odometry
from avai_messages.msg import BoundingBoxWithRealCoordinates
from avai_messages.msg import BoundingBoxesWithRealCoordinates
from geometry_msgs.msg import _pose_with_covariance
from geometry_msgs.msg import _twist_with_covariance

from rcl_interfaces.msg import SetParametersResult

from yolov5.utils.plots import Annotator
import numpy as np

import datetime
import os

import queue

class MappingNode(Node):
    def __init__(self):
        self.bboxesWithRealCoordinates_subscriber = self.create_subscription(BoundingBoxesWithRealCoordinates, '/bboxes_realCoords', self.bbox_callback, 10)

        self.odometry_subscriber = self.create_subscription(_odometry, '/odom', self.odometry_callback, 10)

        self.classes = ['blue', 'orange', 'yellow']

        self.currentMap = [] #entries denote objects in our map, each object consists of xy coordinates and a corresponding class


    def bbox_callback(self, msg):
        newMap = []

        for bbox in msg.bboxes:
            realCoordinates = bbox.real_coords
            cls = int(bbox.cls)

            newMap.append([realCoordinates, cls])

        self.currentMap = newMap

    def odometry_callback(self, msg):
        pose = msg.pose
        twist = msg.twist

def main(args=None):
    rclpy.init(args=args)

    node = MappingNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()