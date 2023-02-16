import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
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
        super().__init__('sensor_fusion_node')
        self.bridge = CvBridge()
        self.boundingBox_subscriber = self.create_subscription(BoundingBoxes, '/bboxes', self.bbox_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)

        self.bboxWithRealCoords_publisher = self.create_publisher(BoundingBoxesWithRealCoordinates, '/bboxes_realCoords', 10)

        self.bboxes = []
        self.lidar = []
        self.classes = ['blue', 'orange', 'yellow']

        self.sensorFusionClock = self.create_timer(0.1, self.attempt_sensor_fusion)
        self.msgCleanupClock = self.create_timer(1, self.remove_old_messages)

        print("Sensor Fusion Node started!")


    def bbox_callback(self, msg):
        self.get_logger().info('Receiving bbox')
        self.bboxes = np.append(self.bboxes, msg)

    def lidar_callback(self, laser_scan):
        self.get_logger().info('Receiving lidar frame')
        self.lidar = np.append(self.lidar, laser_scan)

    def message_distance(self, timestampA, timestampB):
        totalTimeA = timestampA.sec + timestampA.nanosec * 10e-9
        totalTimeB = timestampB.sec + timestampB.nanosec * 10e-9
        totalDistance = totalTimeB - totalTimeA
        return totalDistance

    def remove_old_messages(self):
        currentStamp = self.get_clock().now().to_msg()
        self.bboxes = np.array([bboxMsg for bboxMsg in self.bboxes if
                                             self.message_distance(bboxMsg.header.stamp, currentStamp) < 10])
        self.lidar = np.array([lidarMsg for lidarMsg in self.lidar if
                                                 self.message_distance(lidarMsg.header.stamp, currentStamp) < 10])

    def attempt_sensor_fusion(self):
        selectedBboxesMsg = None
        for receivedBboxes in np.flip(self.bboxes):
            closestLidarMsg = None
            closestDistance = np.inf
            for receivedLidar in np.flip(self.lidar):
                msgDistance = self.message_distance(receivedBboxes.header.stamp, receivedLidar.header.stamp)
                if msgDistance < closestDistance:
                    closestLidarMsg = receivedLidar
                    selectedBboxesMsg = receivedBboxes
                    break
            if closestLidarMsg:
                self.sensor_fusion(selectedBboxesMsg.bboxes, closestLidarMsg)
                np.delete(self.lidar, np.where(self.lidar == closestLidarMsg))
                np.delete(self.bboxes, np.where(self.lidar == selectedBboxesMsg))
                break


    def sensor_fusion(self, bboxes, lidar):
        clustered_lidar = self.clusterLidarPoints(lidar)
        print("Clustered points: ", clustered_lidar)

        matchedBBoxes = []
        for bbox in bboxes:
            bestMatch = None
            for clusterAngle, clusterDistance in clustered_lidar:
                if bestMatch is None:
                    bestMatch = (clusterAngle, clusterDistance)
                elif self.distanceToBoxCenter(bbox, clusterAngle) < self.distanceToBoxCenter(bbox, bestMatch[0]):
                    bestMatch = (clusterAngle, clusterDistance)

            if not bestMatch is None:
                print("bestMatch: ", bestMatch)
                bboxPos = self.polarToCartesianMirrored(clusterAngle + 59, clusterDistance)
                bboxMsg = BoundingBoxWithRealCoordinates()
                bboxMsg.image_coords = bbox.coordinates
                bboxMsg.conf = bbox.conf
                bboxMsg.cls = bbox.cls
                bboxMsg.real_coords = bboxPos


                matchedBBoxes.append(bboxMsg)

        BBoxesMsg = BoundingBoxesWithRealCoordinates()
        BBoxesMsg.header = Header()
        BBoxesMsg.header.stamp = self.get_clock().now().to_msg()
        BBoxesMsg.bboxes = matchedBBoxes
        self.bboxWithRealCoords_publisher.publish(BBoxesMsg)

        self.get_logger().info('Publishing bounding boxes with real coordinates')

    def angleToPixel(self, angle):
        return (angle / 62.0) * 640
    def distanceToBoxCenter(self, bbox, lidar_angle):
        return np.abs((bbox.coordinates[0] + bbox.coordinates[2])/2 - self.angleToPixel(lidar_angle))

    def clusterLidarPoints(self, lidar):
        # reads the lidar data and clusters the points in the camera fov
        # returns an array of points as (middle_point in degree, distance)
        # (31, 1.5) means right in the center of the camera there is a object 1,5m away
        scan_fov = lidar.ranges[149:212]
        index = -1
        TOLERANCE = 0.01
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
            # results.append((round(end - start), mean))
            results.append((round((end + start) / 2), mean))
        return results
    def polarToCartesianMirrored(self, angle, distance):
        angle = (angle/180.0)*np.pi
        x = -np.cos(angle) * distance
        y = np.sin(angle) * distance
        return [x,y]


def main(args=None):
    rclpy.init(args=args)

    node = SensorFusionNode()
    rclpy.spin(node)

    node.destroyc_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()