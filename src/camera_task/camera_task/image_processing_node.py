import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes

import numpy as np

import tensorflow as tf
import tflite_runtime.interpreter as tflite

import argparse


class ImageProcessingNode(Node):

    def __init__(self, cone_detection, edge_tpu):
        super().__init__('image_processing_node')
        print("Starting image processing node!")
        self.bridge = CvBridge()
        # subscriber for raw img data
        self.raw_image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
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
                self.interpreter = tflite.Interpreter('src/camera_task/models/best-int8_edgetpu.tflite',
                                                       experimental_delegates=[
                                                           tflite.load_delegate('libedgetpu.so.1')])
            else:
                print("Loading normal model!")
                self.interpreter = tflite.Interpreter('src/camera_task/models/best-fp16.tflite')
            self.interpreter.allocate_tensors()

        print("Image Processing Node started!")

    def xywh2xyxy(self, boxes):
        xyxy = np.zeros((len(boxes), 4))
        xyxy[:, 0] = boxes[:, 0] - (boxes[:, 2] / 2)
        xyxy[:, 1] = boxes[:, 1] - (boxes[:, 3] / 2)
        xyxy[:, 2] = boxes[:, 0] + (boxes[:, 2] / 2)
        xyxy[:, 3] = boxes[:, 1] + (boxes[:, 3] / 2)
        return xyxy

    def normalizedBoxesToImageSize(self, boxes, width, height):
        denormalizedBoxes = np.zeros((len(boxes), 4))
        denormalizedBoxes[:, 0] = boxes[:, 0] * width
        denormalizedBoxes[:, 2] = boxes[:, 2] * height
        denormalizedBoxes[:, 1] = boxes[:, 1] * width
        denormalizedBoxes[:, 3] = boxes[:, 3] * height
        return denormalizedBoxes

    #Necessary step for working with edge tpu data which returns values between 0 and 127
    def normalizeBoxes(self, boxes):
        edge_tpu_max_value = 145.0
        normalizedBoxes = np.zeros((len(boxes), 4))
        normalizedBoxes[:, 0] = boxes[:, 0] / edge_tpu_max_value
        normalizedBoxes[:, 2] = boxes[:, 2] / edge_tpu_max_value
        normalizedBoxes[:, 1] = boxes[:, 1] / edge_tpu_max_value
        normalizedBoxes[:, 3] = boxes[:, 3] / edge_tpu_max_value
        return normalizedBoxes

    def clipBoxes(self, boxes):
        clippedBoxes = np.zeros((len(boxes), 4))
        clippedBoxes[:, 0] = np.clip(boxes[:, 0], 0, 1)
        clippedBoxes[:, 2] = np.clip(boxes[:, 2], 0, 1)
        clippedBoxes[:, 1] = np.clip(boxes[:, 1], 0, 1)
        clippedBoxes[:, 3] = np.clip(boxes[:, 3], 0, 1)
        return clippedBoxes


    def callback(self, msg):
        self.get_logger().info(f"Received new raw image!")

        original_image = self.bridge.imgmsg_to_cv2(msg)
        original_image = cv2.resize(original_image, (self.targetWidth, self.targetHeight))
        self.last_received_image = original_image

        if self.cone_detection:
            prediction = self.image_to_prediction(original_image)
            
            bbox_msg = self.prediction_to_bounding_box_msg(prediction)
            #bbox_msg.header = Header()
            #bbox_msg.header.stamp = msg.header.stamp
            self.bounding_box_publisher.publish(bbox_msg)
            self.get_logger().info('Publishing bounding boxes')
        else:
            bbox_msg = BoundingBoxes()
            self.bounding_box_publisher.publish(bbox_msg)
            self.get_logger().info('no bounding boxes')
        # convert image to compressed image
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(original_image)
        compressed_image.header = Header()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
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
        bbox_msg.header = Header()
        bbox_msg.header.stamp = self.get_clock().now().to_msg()
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

    # In this section the image is pushed through the yolov5 network.
    # The resulting bounding boxes are reformatted and non-max-suppression is applied to filter out unimportant boxes.
    def image_to_prediction(self, original_image):
        if self.cone_detection:
            # preparing image for yolov5 network
            prepared_image = self.prepare_image_for_model(original_image)

            input_details = self.interpreter.get_input_details()[0]
            output_details = self.interpreter.get_output_details()[0]
            self.interpreter.set_tensor(input_details['index'], prepared_image)
            print("Start invoke:" , self.get_clock().now().seconds_nanoseconds())
            self.interpreter.invoke()
            print("End invoke:", self.get_clock().now().seconds_nanoseconds())
            prediction = self.interpreter.get_tensor(output_details['index'])
            prediction = prediction[0]
            
            boxes = prediction[:, :4]
            boxes = self.xywh2xyxy(boxes)
            scores = prediction[:, 4]
            cls = [np.argmax(score) for score in prediction[:, 5:]]

            if self.edge_tpu:
                boxes = self.normalizeBoxes(boxes)

            boxes = self.clipBoxes(boxes)

            boxes = self.normalizedBoxesToImageSize(boxes, 640, 640)

            selected_indices = tf.image.non_max_suppression(boxes, scores/200, max_output_size=10, iou_threshold=0.15, score_threshold=0.35)

            selected_boxes = np.array(tf.gather(boxes, selected_indices))
            selected_cls = np.array(tf.gather(cls, selected_indices))
            selected_scores = np.array(tf.gather(scores, selected_indices))

            bboxes = list(zip(selected_boxes, selected_scores, selected_cls))
            return bboxes

    def get_last_received_image(self):
        return self.last_received_image


def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Image Processing Node')
    parser.add_argument('-c', '--cone_detection', type=str2bool, default=True, help='Enable cone detection')
    parser.add_argument('-e', '--edge_tpu', type=str2bool, default=True, help='Enable Edge TPU')
    args = parser.parse_args()

    print(f'Cone detection {"enabled" if args.cone_detection else "disabled"}')
    print(f'Edge TPU {"enabled" if args.edge_tpu else "disabled"}')
    node = ImageProcessingNode(args.cone_detection, args.edge_tpu)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
