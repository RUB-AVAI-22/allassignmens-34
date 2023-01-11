import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes
from avai_messages.msg import BoundingBoxWithRealCoordinates
from avai_messages.msg import BoundingBoxesWithRealCoordinates
from rcl_interfaces.msg import SetParametersResult


from yolov5.utils.plots import Annotator
import numpy as np

import datetime
import os

import queue


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        # timer
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.control_callback)
        # video subscriber
        self.bridge = CvBridge()
        self.boundingBox_subscriber = self.create_subscription(BoundingBoxes, '/bboxes', self.bbox_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)

        self.bboxWithRealCoords_publisher = self.create_publisher(BoundingBoxesWithRealCoordinates, '/bboxes_realCoords', 10)

        self.bbox_queue = queue.SimpleQueue()
        self.classes = ['blue', 'orange', 'yellow']

        self.timer = self.create_timer(1 / 20, self.matchClusterToBoundingBoxes)
        self.currentLidarClusters = []

        print("Node started!")


    def bbox_callback(self, msg):
        bboxes = []
        for bbox in msg.bboxes:
            extractedBbox = []
            extractedBbox.append(bbox.coordinates)
            extractedBbox.append(bbox.conf)
            extractedBbox.append(bbox.cls)
            bboxes.append(extractedBbox)


        self.bbox_queue.put(bboxes)

    def lidar_callback(self, laser_scan):
        #self.get_logger().info('Receiving lidar frame')
        # reads the lidar data and clusters the points in the camera fov
        # returns an array of points as (middle_point in degree, distance)
        # (31, 1.5) means right in the center of the camera there is a object 1,5m away
        print("Lidar Data received")
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
                new_mean = mean + (distance - mean) / (current_degree - start + 1)
                clusters[index] = (start, current_degree, new_mean)
                last_value = distance
        for cluster in clusters:
            start, end, mean = cluster
            #results.append((round(end - start), mean))
            results.append((round((end+start)/2), mean))
        print("Objects found my Lidar: ", results)
        self.currentLidarClusters = results

    def polarToCartesian(self, angle, distance):
        x = np.cos(angle) * distance
        y = np.sin(angle) * distance
        return [x,y]

    def matchClusterToBoundingBoxes(self):
        if self.bbox_queue.empty():
            return
        if self.currentLidarClusters == []:
            return

        bboxes = self.bbox_queue.get()

        matchedBBoxes = []

        for clusterAngle, clusterDistance in self.currentLidarClusters:
            clusterPixelApprox = (clusterAngle/64.0)*640
            for bbox in bboxes:
                if clusterPixelApprox > bbox[0] and clusterPixelApprox < bbox[2]:
                    bboxPos = self.polarToCartesian(clusterAngle, clusterDistance)
                    matchedBBoxes += [*bbox, bboxPos]
                    break

        self.bboxWithRealCoords_publisher.publish(matchedBBoxes)



def main(args=None):
    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

    rclpy.init(args=args)

    node = SensorFusionNode()
    rclpy.spin(node)

    node.destroyc_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()