import numpy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg._laser_scan import LaserScan

class LidarSensorNode(Node):

    def __init__(self):
        super().__init__('lidar_sensor_node')

        #Subscribe to Lidar
        self.lidar_sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.debug_showLaserScans, qos_profile_sensor_data)

        #Publisher for Clustered Points
        self.publisher = self.create_publisher(LaserScan, '/lidar_data', 10)


    def debug_showLaserScans(self, laser_scan):
        print(time.time())
        print(laser_scan)


def main(args=None):
    print('lidar_sensor_node startet')
    rclpy.init(args=args)

    node = LidarSensorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
