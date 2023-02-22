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

    def test_nothing_receive_after_startup(self):
        assert self.map_display_node.get_logger() is None
#        assert self.map_display_node.map_subscriber.
if __name__ == '__main__':
    unittest.main()
