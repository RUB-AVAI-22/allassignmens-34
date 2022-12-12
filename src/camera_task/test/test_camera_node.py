import unittest
import rclpy
import image_display_node

from geometry_msgs.msg import Twist

from rclpy.node import Node
class CameraNodeTest(unittest.TestCase):

    def setUp(self):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    def test_something(self):
        self.assertEqual(True, False)  # add assertion here


if __name__ == '__main__':
    unittest.main()
