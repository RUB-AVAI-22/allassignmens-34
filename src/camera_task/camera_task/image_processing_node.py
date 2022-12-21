import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes

import numpy as np

from yolov5.models.common import DetectMultiBackend

import tensorflow as tf
import tflite_runtime.interpreter as tflite

import torch


class ImageProcessingNode(Node):

    def __init__(self, cone_detection, edge_tpu):
        super().__init__('image_processing_node')
        print("Starting image processing node!")
        self.bridge = CvBridge()
        # subscriber for raw img data
        self.raw_image_subscriber = self.create_subscription(Image, '/raw_image', self.callback, 10)
        # publisher for compressed img data
        self.compressed_image_publisher = self.create_publisher(CompressedImage, '/proc_img', 10)
        # publisher for bounding box data
        self.bounding_box_publisher = self.create_publisher(BoundingBoxes, '/bboxes', 10)

        self.targetWidth = 640
        self.targetHeight = 640

        self.last_received_image = None
        self.cone_detection = cone_detection
        self.edge_tpu = edge_tpu
        # Initializing the yolov5 model
        if cone_detection:
            self.classes = ['blue', 'orange', 'yellow']
            # edge_tpu model only runs on tpu so a different model has to be loaded when not run on tpu
            if edge_tpu:
                print("Loading edge tpu model!")
                self.interpreter = tflite.Interpreter('models/best-int8_edgetpu.tflite',
                                                       experimental_delegates=[
                                                           tflite.load_delegate('libedgetpu.so.1')])
                self.interpreter.allocate_tensors()
                print("Successfully loaded model!")
            else:
                print("Loading normal model!")
                self.interpreter = tflite.Interpreter('models/best-fp16.tflite')
                self.interpreter.allocate_tensors()
                print("Successfully loaded model!")

    def xywh2xyxy(self, boxes):
        if boxes is None:
            return None
        if not len(boxes.shape) == 2:
            return np.empty(0)
        if boxes.shape[0] == 0:
            return np.empty(0)
        if not boxes.shape[1] == 4:
            return np.empty(0)


        xyxy = np.zeros((len(boxes), 4))
        xyxy[:, 0] = np.clip(boxes[:, 0] - (boxes[:, 2] / 2), 0, 1)
        xyxy[:, 1] = np.clip(boxes[:, 1] - (boxes[:, 3] / 2), 0, 1)
        xyxy[:, 2] = np.clip(boxes[:, 0] + (boxes[:, 2] / 2), 0, 1)
        xyxy[:, 3] = np.clip(boxes[:, 1] + (boxes[:, 3] / 2), 0, 1)
        return xyxy

    def normalizedBoxesToImageSize(self, boxes, width, height):
        denormalizedBoxes = np.zeros((len(boxes), 4))
        denormalizedBoxes[:, 0] = boxes[:, 0] * width
        denormalizedBoxes[:, 2] = boxes[:, 2] * width
        denormalizedBoxes[:, 1] = boxes[:, 1] * height
        denormalizedBoxes[:, 3] = boxes[:, 3] * height
        return denormalizedBoxes

    def callback(self, msg):
        self.get_logger().info(f"Received new raw image!")

        original_image = self.bridge.imgmsg_to_cv2(msg)
        original_image = cv2.resize(original_image, (self.targetWidth, self.targetHeight))
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
        bboxes = prediction

        # publish bounding boxes
        bbox_msg = BoundingBoxes()
        msg_data = []
        for xyxy, conf, cls in bboxes:
            bbox = BoundingBox()
            bbox.coordinates = [float(tensor) for tensor in xyxy]
            bbox.conf = float(conf)
            bbox.cls = float(cls)
            msg_data.append(bbox)
        bbox_msg.bboxes = msg_data
        return bbox_msg

    def prepare_image_for_model(self, original_image):
        prepared_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
        prepared_image = np.expand_dims(prepared_image, axis=0)

        if self.edge_tpu:
            prepared_image = prepared_image.astype(np.uint8)
        else:
            prepared_image = prepared_image.astype(np.float32)
            prepared_image /= 255

        return prepared_image

    def image_to_prediction(self, original_image):
        if self.cone_detection:
            # preparing image for yolov5 network
            prepared_image = self.prepare_image_for_model(original_image)

            input_details = self.interpreter.get_input_details()[0]
            output_details = self.interpreter.get_output_details()[0]
            self.interpreter.set_tensor(input_details['index'], prepared_image)
            self.interpreter.invoke()
            prediction = self.interpreter.get_tensor(output_details['index'])
            prediction = prediction[0]

            boxes = prediction[:, :4]
            boxes = self.xywh2xyxy(boxes)
            scores = prediction[:, 4]
            cls = [np.argmax(score) for score in prediction[:, 5:]]

            boxes = self.normalizedBoxesToImageSize(boxes, 640, 640)

            selected_indices = tf.image.non_max_suppression(boxes, scores, max_output_size=10, iou_threshold=0.25)
            selected_boxes = np.array(tf.gather(boxes, selected_indices))
            selected_cls = np.array(tf.gather(cls, selected_indices))
            selected_scores = np.array(tf.gather(scores, selected_indices))
            bboxes = list(zip(selected_boxes, selected_scores, selected_cls))

            return bboxes

    def get_last_received_image(self):
        return self.last_received_image


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode(True, True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
