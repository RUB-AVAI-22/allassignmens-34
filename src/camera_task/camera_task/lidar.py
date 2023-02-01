import numpy as np
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

    def groupCone(self,lst):
        new_list = []
        for i,j in zip(lst[0],lst[1:][0]):
            if j - i > 1:
                new_list.append(lst[:lst.index(j)])
                lst = lst[lst.index(j):]
        new_list.append(lst)
        return new_list

    def cluster_points_in_fov(self, lidar):
        #reads the lidar data and clusters the points in the camera fov
        #returns an array of points as (middle_point in degree, distance)
        #(31, 1.5) means right in the center of the camera there is a object 1,5m away
        left = []
        right = []
        
        laser_scan_list = lidar.ranges
        #print(f"\r{laser_scan_list}\n\n\n", end='')
        left = laser_scan_list[0:180]
        right = laser_scan_list[181:359]
        right.reverse()

        scan_fov = lidar.ranges[149:212]
        index = -1
        TOLERANCE = 0.05
        last_value = 0
        clusters = []
        results = []
        for current_degree, distance in enumerate(left):
            if abs(distance - last_value) > distance * TOLERANCE and distance != 0:
                clusters.append((current_degree, current_degree, distance))
                last_value = distance
                index += 1
         
                start, end, mean = clusters[index]
                new_mean = mean + (distance - mean) / (current_degree - start + 1)
                clusters[index] = (start, current_degree, new_mean)
                last_value = distance
        for cluster in clusters:
            start, end, mean = cluster
            results.append((round((end + start) / 2), mean))
        # return results

        #res_left = [(i,left[i])for i in range(len(left)) if  left[i] != float("inf")]
        #res_right = [[i,right[i]] for i in range(len(right)) if  right[i] != float("inf")]
        #data = pickle.dumps(results)
        print("\n\n",results)
        #left_grouped = self.groupCone(left)
        #right_grouped = self.groupCone(res_right)
        #print("left",left)
        #print(len(left))

        #print(right_grouped)
        #print(len(left))
        #degree = laser_scan.angle_increment*(180/math.pi)
        #print(degree)
        #print(len(laser_scan.ranges))




def main(args=None):
    print('lidar_sensor_node startet')
    rclpy.init(args=args)

    node = LidarSensorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
