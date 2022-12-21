import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes
from rcl_interfaces.msg import SetParametersResult


from yolov5.utils.plots import Annotator
import numpy as np

import datetime
import os

import queue

import math
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi



class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        # timer
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.control_callback)
        # video subscriber
        self.bridge = CvBridge()
        self.proccessedImage_subscriber = self.create_subscription(CompressedImage, '/proc_img', self.video_callback, 10)
        self.boundingBox_subscriber = self.create_subscription(BoundingBoxes, '/bboxes', self.bbox_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 0)

        self.annotatedImage_publisher = self.create_publisher(CompressedImage, '/annotated_img', 10)

        self.bbox_queue = queue.SimpleQueue()
        self.image_queue = queue.SimpleQueue()
        self.classes = ['blue', 'orange', 'yellow']

        self.timer = self.create_timer(1 / 20, self.annotation)

        print("Node started!")

    def video_callback(self, msg):
        if frame:
            current_frame = self.bridge.compressed_imgmsg_to_cv2(frame)
            self.image_queue.put(current_frame)

    def bbox_callback(self, msg):
        bboxes = []
        dictonary = {}
        for bbox in msg.bboxes:
            extractedBbox = []
            extractedBbox.append(bbox.coordinates)
            extractedBbox.append(bbox.conf)
            extractedBbox.append(bbox.cls)
            bboxes.append(extractedBbox)
        
        self.bbox_queue.put(bboxes)
        

    def lidar_callback(self, msg):
        self.get_logger().info('Receiving lidar frame')
        if msg:

            angles = msg[:,1]
            distances = msg[:,0]

            angles = np.array(angles)
            distances = np.array(distances)
            
            ox = np.cos(ang) * dist
            oy = np.sin(ang) * dist
            
            plt.figure(figsize=(6,10))
            plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-") # lines from 0,0 to the
            plt.axis("equal")
            bottom, top = plt.ylim()  # return the current ylim
            plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
            plt.grid(True)
            plt.show()
            
            for message in msg:
                if self.bbox_queue.get() >= 1:
                    bboxs = self.bbox_queue.get()
                    for bbox in bboxs:
                        bbox_range = message[0] * 10.3
                        dictonary = {1: 'blue', 2: 'orange', 3: 'yellow'}
                        if bbox.coordinates[0] <= bbox_range and bbox.coordinates.[2] >= bbox_range:
                            print(f'cone {dictonary[bbox.cls + 1]} angel = {message[0]}, distance = {message[1]}')
                

    def annotation(self):
        if self.bbox_queue.qsize() >= 1 and self.image_queue.qsize() >= 1:
            image = self.image_queue.get()
            bboxes = self.bbox_queue.get()


            # annotating image with detected bounding boxes
            annotator = Annotator(image)
            for *xyxy, conf, cls in bboxes:
                if conf > 0.7:
                    annotator.box_label(xyxy[0], f'{self.classes[int(cls)]} {conf:.2f}')

            annotated_image = annotator.result()


            annotated_image = cv2.resize(annotated_image, (640, 480))
            self.annotatedImage_publisher.publish(annotated_image)


def main(args=None):
    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

    rclpy.init(args=args)

    node = SensorFusionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
