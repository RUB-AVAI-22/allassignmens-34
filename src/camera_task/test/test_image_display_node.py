import unittest

import cv2
import rclpy
from src.camera_task.camera_task.image_display_node import ImgDisplayNode

from geometry_msgs.msg import Twist

from rclpy.node import Node


class ImageDisplayNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.image_display_node = ImgDisplayNode()

    def tearDown(self) -> None:
        self.image_display_node.destroy_node()

    def test_nothing_received_after_startup(self):
        #No image/bbox received and queues are empty
        assert None == self.image_display_node.get_last_received_image()
        assert None == self.image_display_node.get_last_received_bbox()
        assert self.image_display_node.image_queue.empty()
        assert self.image_display_node.bbox_queue.empty()

    def test_window_open_after_startup(self):
        assert cv2.getWindowProperty('frame', cv2.WND_PROP_VISIBLE) >= 1



if __name__ == '__main__':
    unittest.main()
