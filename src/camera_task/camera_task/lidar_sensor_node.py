import numpy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import pickle

from sensor_msgs.msg._laser_scan import LaserScan


class LidarSensorNode(Node):

    def __init__(self):
        super().__init__('lidar_sensor_node')

        # Subscribe to Lidar
        self.lidar_sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.debug_showLaserScans,
                                                                qos_profile_sensor_data)

        # Publisher for Clustered Points
        self.publisher = self.create_publisher(bytes, '/lidar_data', 10)

    def cluster_points_in_fov(self, laser_scan):
        #reads the lidar data and clusters the points in the camera fov
        #returns an array of points as (middle_point in degree, distance)
        #(31, 1.5) means right in the center of the camera there is a object 1,5m away
        scan_fov = laser_scan.range[149:212]
        index = -1
        TOLERANCE = 0.05
        last_value = 0
        clusters = []
        results = []
        for current_degree, distance in enumerate(scan_fov):
            if abs(distance - last_value) > distance * TOLERANCE and distance != 0:
                clusters.append((current_degree, current_degree, distance))
                last_value = distance
                index += 1
            elif distance != 0:
                start, end, mean = clusters[index]
                new_mean = mean + (distance - mean)/(current_degree - start + 1)
                clusters[index] = (start, current_degree, new_mean)
                last_value = distance
        for cluster in clusters:
            start, end, mean = cluster
            results.append((round(end - start), mean))

        data = pickle.dumps(results)
        self.publisher.publish(data)




    def debug_showLaserScans(self, laser_scan):
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
