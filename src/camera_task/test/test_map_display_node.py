import unittest
import rclpy

from src.camera_task.camera_task.map_display_node import MapDisplayNode
from unittest import TestCase

class MapDisplayNodeTest(TestCase):
    @classmethod
    def setUp(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.map_display_node = MapDisplayNode()

    def tearDown(self):
        self.map_display_node.destroy_node()


if __name__ == '__main__':
    unittest.main()
