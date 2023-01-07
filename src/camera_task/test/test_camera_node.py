import unittest
import rclpy
from src.camera_task.camera_task.camera_node import CameraNode

from unittest import TestCase

class CameraNodeTest(TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.camera_node = CameraNode(False)

    def tearDown(self) -> None:
        self.camera_node.destroy_node()

    def test_video_stream_open_after_startup(self):
        assert self.camera_node.cap.read()[0]

    def test_fps(self):
        assert self.camera_node.param_fps(-1)
        assert self.camera_node.param_fps()


if __name__ == '__main__':
    unittest.main()
