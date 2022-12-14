import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes

import datetime
import os

import numpy as np

from yolov5.utils.plots import Annotator
from yolov5.utils.general import non_max_suppression
from yolov5.models.common import DetectMultiBackend

import torch


class ImageProcessingNode(Node):

    def __init__(self, cone_detection, edge_tpu):
        super().__init__('image_processing_node')

        self.path = "/home/ubuntu/allassignmens-34/src/camera_task"

        self.bridge = CvBridge()
        # subscriber for raw img data
        self.raw_image_subscriber = self.create_subscription(Image, '/raw_image', self.callback, 10)
        # publisher for compressed img data
        self.compressed_image_publisher = self.create_publisher(CompressedImage, '/proc_img', 10)
        # publisher for bounding box data
        self.bounding_box_publisher = self.create_publisher(BoundingBoxes, '/bboxes', 10)

        self.last_received_image = None
        self.cone_detection = cone_detection
        self.edge_tpu = edge_tpu
        # Initializing the yolov5 model
        if cone_detection:
            self.targetWidth = 640
            self.targetHeight = 640
            self.classes = ['blue', 'orange', 'yellow']
            # edge_tpu model only runs on tpu so a different model has to be loaded when not run on tpu
            if edge_tpu:
                self.model = DetectMultiBackend(self.path + '/models/best-uint8_edgetpu.tflite')
            else:
                self.model = DetectMultiBackend(self.path + '/models/best-fp16.tflite')
            self.model.warmup(imgsz=(1, self.targetWidth, self.targetHeight, 3))

    def callback(self, msg):
        self.get_logger().info(f"Received new raw image!")

        original_image = self.bridge.imgmsg_to_cv2(msg)
        self.last_received_image = original_image

        if self.cone_detection:
            prediction = self.image_to_prediction(original_image)

            bbox_msg = self.prediction_to_bounding_box_msg(prediction)
            self.bounding_box_publisher.publish(bbox_msg)

        # convert image to compressed image
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(original_image)
        # publish compressed image
        self.compressed_image_publisher.publish(compressed_image)

    def prediction_to_bounding_box_msg(self, prediction):
        bboxes = prediction[0]

        # publish bounding boxes
        bbox_msg = BoundingBoxes()
        msg_data = []
        for *xyxy, conf, cls in bboxes:
            bbox = BoundingBox()
            bbox.coordinates = [float(tensor) for tensor in xyxy]
            bbox.conf = float(conf)
            bbox.cls = float(cls)
            msg_data.append(bbox)
        bbox_msg.bboxes = msg_data
        return bbox_msg

    def prepare_image_for_model(self, original_image):
        original_image = cv2.resize(original_image, (self.targetWidth, self.targetHeight))
        if self.edge_tpu:
            prepared_image = torch.from_numpy(
                cv2.dnn.blobFromImage(cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB), 1,
                                      (self.targetWidth, self.targetHeight),
                                      crop=False).astype(np.uint8)).to(self.model.device)
        else:
            prepared_image = torch.from_numpy(
                cv2.dnn.blobFromImage(cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB), 1 / 255,
                                      (self.targetWidth, self.targetHeight),
                                      crop=False).astype(np.float32)).to(self.model.device)

        return prepared_image

    def image_to_prediction(self, original_image):
        if self.cone_detection:
            # preparing image for yolov5 network
            prepared_image = self.prepare_image_for_model(original_image)

            # running the yolov5 network on the image
            prediction = self.model(prepared_image)
            prediction = non_max_suppression(prediction, 0.25, 0.45, [0, 1, 2], False, max_det=1000)

            return prediction

    def get_last_received_image(self):
        return self.last_received_image


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode(True, False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
