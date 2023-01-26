import numpy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import pickle
import math
from sensor_msgs.msg._laser_scan import LaserScan
from nav_msgs.msg import Odometry
class LidarSensorNode(Node):

    def __init__(self):
        super().__init__('lidar_sensor_node')

        # Subscribe to Lidar
        self.lidar_sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.cluster_points_in_fov,
                                                                qos_profile_sensor_data)

        # Publisher for Clustered Points
        #self.publisher = self.create_publisher(bytes, '/lidar_data', 10)

    def cluster_points_in_fov(self, laser_scan):
        #reads the lidar data and clusters the points in the camera fov
        #returns an array of points as (middle_point in degree, distance)
        #(31, 1.5) means right in the center of the camera there is a object 1,5m away
        left = []
        right = []
        
        laser_scan_list = laser_scan.ranges.tolist()
        #print(f"\r{laser_scan_list}\n\n\n", end='')
        left = laser_scan_list[0:180]
        right = laser_scan_list[181:359]
        right.reverse()
        res_left = [i for i in range(len(left)) if  left[i] != float("inf")]
        res_right = [i for i in range(len(right)) if  right[i] != float("inf")]
        #print("left",left)
        #print(len(left))
        print(res_right)
        #print(len(left))
        #degree = laser_scan.angle_increment*(180/math.pi)
        #print(degree)
        #print(len(laser_scan.ranges))



    def debug_showLaserScans(self, laser_scan):
        #print(laser_scan)
        pass


def main(args=None):
    print('lidar_sensor_node startet')
    rclpy.init(args=args)

    node = LidarSensorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
