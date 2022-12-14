import numpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg._laser_scan import LaserScan

class ImgDisplayNode(Node):

    def __init__(self):
        super().__init__('lidar_sensor_node')

        #Subscribe to Lidar
        self.lidar_sensor_subscriber = self.create_subscription(LaserScan, '/sensor_msg', self.callback, 10)

        #Publisher for Clustered Points
        self.publisher = self.create_publisher(int, '/lidar_data', 10)


    def debug_showLaserScans(self, laser_scan):
        print(laser_scan)
