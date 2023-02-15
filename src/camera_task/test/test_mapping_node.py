import unittest
import rclpy

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


if __name__ == '__main__':
    unittest.main()
