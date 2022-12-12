import unittest
import cv2
import rclpy
import torch
import os
import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes

from src.camera_task.camera_task.image_processing_node import ImageProcessingNode

from yolov5.utils.general import non_max_suppression
from yolov5.models.common import DetectMultiBackend

from unittest import TestCase


# Unit Tests
class ImageProcessingNodeTest(TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.image_processing_node = ImageProcessingNode(True, False)
        self.test_image = cv2.imread(os.path.dirname(__file__) + '/picture.jpeg')
        self.test_model = DetectMultiBackend(os.path.dirname(__file__) + '/../models/best-fp16.tflite')

        self.prepared_image = cv2.resize(self.test_image, (640, 640))
        self.prepared_image = torch.from_numpy(
            cv2.dnn.blobFromImage(cv2.cvtColor(self.prepared_image, cv2.COLOR_BGR2RGB), 1,
                                  (640, 640),
                                  crop=False).astype(np.uint8)).to(self.model.device)
        self.prediction = self.test_model(self.prepared_image)
        self.prediction = non_max_suppression(self.prediction, 0.25, 0.45, [0, 1, 2], False, max_det=1000)

        bboxes = self.prediction[0]

        # publish bounding boxes
        self.bbox_msg = BoundingBoxes()
        msg_data = []
        for *xyxy, conf, cls in bboxes:
            bbox = BoundingBox()
            bbox.coordinates = [float(tensor) for tensor in xyxy]
            bbox.conf = float(conf)
            bbox.cls = float(cls)
            msg_data.append(bbox)
        self.bbox_msg.bboxes = msg_data

    def tearDown(self) -> None:
        self.image_processing_node.destroy_node()

    def test_image_preprocessing(self):
        prepared_image_from_node = self.image_processing_node.prepare_image_for_model(self.test_image)

        self.assertEqual(self.prepared_image, prepared_image_from_node)

    def test_model_prediction(self):
        prediction_from_node = self.image_processing_node.image_to_prediction(self.test_image)

        self.assertEqual(self.prediction, prediction_from_node)

    def test_bounding_box_message_creation(self):
        bounding_box_msg_from_node = self.prediction_to_bounding_box_msg(
            self.image_processing_node.image_to_prediction(self.test_image))
        self.assertEqual(self.bbox_msg, bounding_box_msg_from_node)


# Integration Tests (maybe move to separate file)
class CameraNodeIntegrationTest(TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.imageProcessingNode = ImageProcessingNode(False, False)
        self.test_image = cv2.imread(os.path.dirname(__file__) + '/picture.jpeg')
        self.mock_publisher = self.create_publisher(Image, '/raw_image', 10)
        self.bridge = CvBridge()

    def test_cameraNode_to_processingNode_connection(self):
        test_image_message = self.bridge.cv2_to_imgmsg(self.test_image)
        self.mock_publisher.publish(test_image_message)

        rclpy.spin_once(self.imageProcessingNode)

        node_received_image = self.imageProcessingNode.last_received_image()

        self.assertEqual(self.test_image, node_received_image)


if __name__ == '__main__':
    unittest.main()
