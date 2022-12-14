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

    def tearDown(self) -> None:
        self.image_processing_node.destroy_node()

    def test_initial_conditions(self):
        assert self.image_processing_node.get_last_received_image() == None


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
        self.mock_publisher = self.imageProcessingNode.create_publisher(Image, '/raw_image', 10)
        self.bridge = CvBridge()

    def test_cameraNode_to_processingNode_connection(self):
        test_image_message = self.bridge.cv2_to_imgmsg(self.test_image)
        self.mock_publisher.publish(test_image_message)

        rclpy.spin_once(self.imageProcessingNode)

        node_received_image = self.imageProcessingNode.get_last_received_image()

        assert (self.test_image == node_received_image).all()


if __name__ == '__main__':
    unittest.main()
