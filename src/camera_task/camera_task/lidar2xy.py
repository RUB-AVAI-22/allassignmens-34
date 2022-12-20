import rclpy
from rclpy.node import Node

import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi

from sensor_msgs.msg import PointCloud2

class PointCloud2XY(Node):
    
    def __ini__(self):
        self.subscriber = self.create_subscription(PointCloud2, '/pointCloud2', self.pointcloud_callback, 10)
        
        # set parameters
        self.param_store_xymap = self.declare_parameter('store_xymap', False)
        self.add_on_set_parameters_callback(self.parameter_callback)

        
    def pointcloud_callback(self, pointCloud):
        if pointCloud:
            print('LiDAR data recieved!')
            
            measures = [line.split(",") for line in pointCloud]
            angles = []
            distances = []
            
            for measure in measures:
                angles.append(float(measure[0]))
                distances.append(float(measure[1]))
            angles = np.array(angles)
            distances = np.array(distances)
            
            ox = np.sin(ang) * dist
            oy = np.cos(ang) * dist
            
            plt.figure(figsize=(6,10))
            plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-") # lines from 0,0 to the
            plt.axis("equal")
            bottom, top = plt.ylim()  # return the current ylim
            plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
            plt.grid(True)
            plt.show()

        
    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'store_xymap':
                self.get_logger().info(f'changed store_xymap from {self.param_store_imgs} to {parameter.value}')
                self.param_store_xymap = parameter.value
                return SetParametersResult(successful=True)
        
        
def main(args=None):
    rclpy.init(args=args)

    node = PointCloud2XY()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

