import unittest
import cv2
import rclpy
import os
import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from src.camera_task.camera_task.image_processing_node import ImageProcessingNode

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
        assert self.image_processing_node.get_last_received_image() is None

    def test_xywh2xyxy_sampleValues(self):
        xywh_sample = np.array([[0.7, 0.4, 0.6, 0.2], [0.8, 0.9, 0.1, 0.1]])
        xyxy_expected = np.array([[0.4, 0.3, 1, 0.5], [0.75, 0.85, 0.85, 0.95]])

        xyxy_result = self.image_processing_node.xywh2xyxy(xywh_sample)

        assert xyxy_expected.shape == xyxy_result.shape
        assert np.testing.assert_array_equal(xyxy_expected.tolist(), xyxy_result.tolist())

    def test_xywh2xyxy_emptySample(self):
        xywh_sample = np.array([])
        xyxy_expected = np.array([])

        xyxy_result = self.image_processing_node.xywh2xyxy(xywh_sample)

        assert xyxy_expected.shape == xyxy_result.shape
        assert xyxy_expected.tolist() == xyxy_result.tolist()

    def test_xywh2xyxy_noneSample(self):
        xywh_sample = None

        xyxy_result = self.image_processing_node.xywh2xyxy(xywh_sample)

        assert xyxy_result is None

    def test_boxNormalization_sampleValue(self):
        width = 100
        height = 100
        normalized_sample = np.array([[0.1, 0.2, 0.3, 0.4]])
        denormalized_expected = np.array([[10, 20, 30, 40]])

        denormalized_result = self.image_processing_node.normalizedBoxesToImageSize(normalized_sample, width, height)

        assert np.array_equal(denormalized_expected, denormalized_result)
        assert np.array_equiv(denormalized_expected, denormalized_result)
        assert np.allclose(denormalized_expected, denormalized_result)


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
