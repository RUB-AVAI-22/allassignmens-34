import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

import datetime

import numpy as np

from yolov5.utils.plots import Annotator
from yolov5.utils.general import non_max_suppression
from yolov5.models.common import DetectMultiBackend

import torch

class ImgProcessingNode(Node):

    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
        # subscriber for raw img data
        self.subscriber_img = self.create_subscription(Image, '/raw_image', self.callback, 10)
        # publisher for compressed img data
        self.publisher_ = self.create_publisher(CompressedImage, '/proc_img', 10)
        self.bbox_publisher = self.create_publisher(Float32MultiArray, '/bboxes', 10)

        #Initializing the yolov5 model
        self.targetWidth = 640
        self.targetHeight = 640
        self.classes = ['blue', 'orange', 'yellow']
        self.model = DetectMultiBackend('best-int8_edgetpu.tflite')
        self.model.warmup(imgsz=(1, self.targetWidth, self.targetHeight, 3))

    def callback(self, msg):
        print(f"received new image {datetime.datetime.now()}")

        original_image = self.bridge.imgmsg_to_cv2(msg)
        
        #preparing image for yolov5 network
        original_image = cv2.resize(original_image, (self.targetWidth, self.targetHeight))
        #prepared_image = torch.from_numpy(cv2.dnn.blobFromImage(cv2.cvtColor(original_image, cv2.COLOR_RGB2BGR), 1, (self.targetWidth, self.targetHeight), crop=False).astype(np.uint8)).to(self.model.device)
        prepared_image = torch.from_numpy(cv2.dnn.blobFromImage(original_image, 1, (self.targetWidth, self.targetHeight), swapRB=True, crop=False).astype(np.uint8)).to(self.model.device)

        #running the yolov5 network on the image
        pred = self.model(prepared_image)
        pred = non_max_suppression(pred, 0.25, 0.45, [0, 1, 2], False, max_det=1000)

        bboxes = pred[0]

        # annotating image with detected bounding boxes
        annotator = Annotator(original_image)
        for *xyxy, conf, cls in bboxes:
            if conf > 0.7:
                annotator.box_label(xyxy, f'{self.classes[int(cls)]} {conf:.2f}')

        annotated_image = annotator.result()
        
        #publish bounding boxes
        bbox_msg = Float32MultiArray()
        #bbox_msg.data = list(pred[0].float().numpy().astype(np.float32).reshape(-1))
        bbox_msg.data = []
        self.bbox_publisher.publish(bbox_msg)

        # convert image to compressed image
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(annotated_image)
        # publish compressed image
        self.publisher_.publish(compressed_image)
        


def main(args=None):
    rclpy.init(args=args)
    node = ImgProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
