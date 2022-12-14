import unittest
import rclpy

from src.camera_task.camera_task.camera_node import CameraNode
from src.camera_task.camera_task.image_processing_node import ImageProcessingNode
from src.camera_task.camera_task.image_display_node import ImgDisplayNode

from unittest import TestCase


class CameraTaskSystemTest(TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.camera_node = CameraNode(False)
        self.image_processing_node = ImageProcessingNode(True, False)
        self.image_display_node = ImgDisplayNode()

    def tearDown(self) -> None:
        self.camera_node.destroy_node()
        self.image_processing_node.destroy_node()
        self.image_display_node.destroy_node()

    def test_image_pipeline(self):
        rclpy.spin_once(self.camera_node)
        rclpy.spin_once(self.image_processing_node)
        rclpy.spin_once(self.image_display_node)
        rclpy.spin_once(self.image_display_node)

        assert self.image_display_node.get_last_received_image() is not None
        assert self.image_display_node.get_last_received_bbox() is not None


if __name__ == '__main__':
    unittest.main()
