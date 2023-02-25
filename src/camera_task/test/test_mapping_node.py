import unittest
import rclpy
import datetime

from src.camera_task.camera_task.mapping_node import MappingNode
from unittest import TestCase

class MappingNodeTest(TestCase):
    @classmethod
    def setUp(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.mapping_node = MappingNode()

    def tearDown(self):
        self.mapping_node.destroy_node()

    def test_message_distance(self):
        assert self.mapping_node.message_distance is None
        assert self.mapping_node.message_distance(self) <=0


if __name__ == '__main__':
    unittest.main()
