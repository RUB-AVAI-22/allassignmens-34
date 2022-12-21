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
        for bbox in msg.bboxes:
            extractedBbox = []
            extractedBbox.append(bbox.coordinates)
            extractedBbox.append(bbox.conf)
            extractedBbox.append(bbox.cls)
            bboxes.append(extractedBbox)


        self.bbox_queue.put(bboxes)

    def lidar_callback(self, laser_scan):
        self.get_logger().info('Receiving lidar frame')
        # reads the lidar data and clusters the points in the camera fov
        # returns an array of points as (middle_point in degree, distance)
        # (31, 1.5) means right in the center of the camera there is a object 1,5m away
        scan_fov = laser_scan.range[149:212]
        index = -1
        TOLERANCE = 0.05
        last_value = 0
        clusters = []
        results = []
        for current_degree, distance in enumerate(scan_fov):
            if abs(distance - last_value) > distance * TOLERANCE and distance != 0:
                clusters.append((current_degree, current_degree, distance))
                last_value = distance
                index += 1
            elif distance != 0:
                start, end, mean = clusters[index]
                new_mean = mean + (distance - mean) / (current_degree - start + 1)
                clusters[index] = (start, current_degree, new_mean)
                last_value = distance
        for cluster in clusters:
            start, end, mean = cluster
            #results.append((round(end - start), mean))
            results.append((round((end+start)/2), mean))

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