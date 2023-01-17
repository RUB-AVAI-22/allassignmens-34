import rclpy
from rclpy.node import Node

from nav_msgs.msg import _odometry
from avai_messages.msg import BoundingBoxWithRealCoordinates
from avai_messages.msg import BoundingBoxesWithRealCoordinates
from geometry_msgs.msg import _pose_with_covariance
from geometry_msgs.msg import _twist_with_covariance

import numpy as np


class MappingNode(Node):
    def __init__(self):
        self.classes = ['blue', 'orange', 'yellow']
        self.currentMap = []  # entries denote objects in our map, each object consists of xy coordinates and a corresponding class

        self.bboxesWithRealCoordinates_subscriber = self.create_subscription(BoundingBoxesWithRealCoordinates,
                                                                             '/bboxes_realCoords', self.bbox_callback,
                                                                             10)
        self.odometry_subscriber = self.create_subscription(_odometry, '/odom', self.odometry_callback, 10)

        self.receivedBboxMsgs = []
        self.receivedOdometryMsgs = []

        self.msgCleanupClock = self.create_timer(1, self.remove_old_messages)
        self.mapUpdateClock = self.create_timer(0.1, self.attempt_map_update)

    def bbox_callback(self, msg):
        np.append(self.receivedBboxes, msg)

    def odometry_callback(self, msg):
        np.append(self.receivedOdometryMsgs, msg)

    def remove_old_messages(self):
        currentStamp = self.get_clock().now().to_msg()
        self.receivedBboxMsgs[:] = np.array([bboxMsg for bboxMsg in self.receivedBboxMsgs if
                                             self.message_distance(bboxMsg.header.stamp, currentStamp) < 1])
        self.receivedOdometryMsgs[:] = np.array([odomMsg for odomMsg in self.receivedOdometryMsgs if
                                                 self.message_distance(odomMsg.header.stamp, currentStamp) < 1])

    def message_distance(self, timestampA, timestampB):
        totalTimeA = timestampA.sec + timestampA.nanosec * 10e-9
        totalTimeB = timestampB.sec + timestampB.nanosec * 10e-9
        totalDistance = totalTimeB - totalTimeA
        return totalDistance

    def attempt_map_update(self):
        selectedBboxesMsg = None
        for receivedBboxes in np.flip(self.receivedBboxMsgs):
            closestOdometryMsg = None
            closestDistance = 0.1
            for receivedOdometry in np.flip(self.receivedOdometryMsgs):
                msgDistance = self.message_distance(receivedBboxes.header.stamp, receivedOdometry.header.stamp)
                if msgDistance < closestDistance:
                    closestOdometryMsg = receivedOdometry
                    selectedBboxesMsg = receivedBboxes
                    break
            if closestOdometryMsg:
                self.update_map(selectedBboxesMsg.bboxes, closestOdometryMsg.pose, closestOdometryMsg.twist)
                np.delete(self.receivedOdometryMsgs, closestOdometryMsg)
                np.delete(self.receivedBboxMsgs, selectedBboxesMsg)
                break

    def update_map(self, bboxes, pose, twist):
        newMap = self.extract_xy_and_cls(bboxes)
        mergedMap = self.empty((len(newMap), 3))
        for i, newObject in enumerate(newMap):
            nearestCurrentObject = None
            smallestDistance = 0.1
            for currentObject in self.currentMap:
                tmpDist = self.mapDistance(newObject, currentObject)
                if newObject[2] == currentObject[2] and tmpDist < smallestDistance:
                    nearestCurrentObject = currentObject
                    smallestDistance = tmpDist
            if nearestCurrentObject:
                mergedMap[i] = self.merge_objects(newObject, nearestCurrentObject)
            else:
                mergedMap[i] = newObject
        self.currentMap = mergedMap

    def merge_objects(self, objectA, objectB):
        if not objectA[2] == objectB[2]:
            return
        mergedObject = [0, 0, 0]
        mergedObject[0] = (objectA[0] + objectB[0]) / 2
        mergedObject[1] = (objectA[1] + objectB[1]) / 2
        mergedObject[2] = objectA[2]
        return mergedObject

    def extract_xy_and_cls(self, bboxes):
        mapObjects = np.empty((len(bboxes), 3))
        mapObjects[:, :2] = bboxes[:, 6:]
        mapObjects[:, 2] = bboxes[:, 5]
        return mapObjects

    def mapDistance(self, objectA, objectB):
        coordsA = objectA[:2]
        coordsB = objectB[:2]
        return np.sqrt((coordsB[0] - coordsA[0]) ** 2 + (coordsB[1] - coordsA[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)

    node = MappingNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
