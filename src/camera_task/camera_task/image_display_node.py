import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes
from rcl_interfaces.msg import SetParametersResult

import message_filters

from yolov5.utils.plots import Annotator
import numpy as np

import datetime
import os

import queue


class ImgDisplayNode(Node):

    def __init__(self):
        super().__init__('image_display_node')
        # timer
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.control_callback)
        # video subscriber
        self.bridge = CvBridge()
        self.image_subscriber = message_filters.Subscriber(CompressedImage, '/proc_img')
        self.bbox_subscriber = message_filters.Subscriber(BoundingBoxes, '/bboxes')

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.image_subscriber, self.bbox_subscriber], 100, 0.1)
        self.synchronizer.registerCallback(self.synchronized_callback)

        self.classes = ['blue', 'orange', 'yellow']

        # set parameters
        self.param_store_imgs = self.declare_parameter('store_imgs', False)
        self.param_imgs_path = self.declare_parameter('imgs_path', 'imgs')

        self.add_on_set_parameters_callback(self.parameter_callback)
        # self.publisher_ = self.create_publisher(String, 'FPS', 10)

        self.sensorFusionClock = self.create_timer(0.1, self.attempt_image_display)
        self.msgCleanupClock = self.create_timer(0.1, self.remove_old_messages)

        # save video
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter('out.mp4', fourcc, 20.0, (640, 480), True)

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

        print("Image Display Node started!")

    def synchronized_callback(self, msg_image, msg_bbox):
        self.get_logger().info('Receiving video frame and bounding boxes')
        self.image_display(msg_bbox.bboxes, self.bridge.compressed_imgmsg_to_cv2(msg_image, 'bgr8'))


    def image_display(self, bboxes, image):
        # annotating image with detected bounding boxes
        annotator = Annotator(image)
        for bbox in bboxes:
            xyxy = bbox.coordinates
            conf = bbox.conf
            cls = bbox.cls
            if conf > 0.7:
                annotator.box_label(xyxy, f'{self.classes[int(cls)]} {conf:.2f}')

        annotated_image = annotator.result()

        annotated_image = cv2.resize(annotated_image, (640, 480))
        cv2.imshow('frame', annotated_image)
        cv2.waitKey(1)

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'store_imgs':
                self.get_logger().info(f'changed store_imgs from {self.param_store_imgs} to {parameter.value}')
                self.param_store_imgs = parameter.value
                return SetParametersResult(successful=True)
            if parameter.name == 'imgs_path':
                self.get_logger().info(f'changed store_imgs from {self.param_imgs_path} to {parameter.value}')
                self.param_imgs_path = parameter.value
                return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = ImgDisplayNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
