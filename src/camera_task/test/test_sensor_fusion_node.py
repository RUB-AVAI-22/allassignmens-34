import unittest
from unittest import TestCase

import rclpy

from src.camera_task.camera_task.sensor_fusion_node import SensorFusionNode


class SensorFusionNodeTest(TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.sensor_fusion_node = SensorFusionNode()

    def tearDown(self) -> None:
        self.sensor_fusion_node.destroy_node()

    def test_receive_topic(self):
        self.assertNotEqual('/bboxes',self.sensor_fusion_node.boundingBox_subscriber)
        self.assertNotEqual('/scan',self.sensor_fusion_node.lidar_subscriber)
        self.assertNotEqual('/odom',self.sensor_fusion_node.odom_subscriber)

    def test_angleToPixel(self):
        self.assertLessEqual(self.sensor_fusion_node.angleToPixel(62),0,'angle_out_of_frame')
        self.assertGreaterEqual(self.sensor_fusion_node.angleToPixel(0),640,'angle_out_of_frame')



if __name__ == '__main__':
    unittest.main()
