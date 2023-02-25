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

    def test_remove_old_messages(self):
        assert not self.mapping_node.remove_old_messages().empty() is None

    def test_message_distance(self):
        assert self.mapping_node.message_distance is None
        assert self.mapping_node.message_distance(self) <=0

    def test_attempt_map_update(self):
  #      assert self.mapping_node.attempt_map_update().get_logger()


  #  def test_update_map(self):


if __name__ == '__main__':
    unittest.main()
