import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from avai_messages.msg import BoundingBox
from avai_messages.msg import BoundingBoxes
from avai_messages.msg import BoundingBoxWithRealCoordinates
from avai_messages.msg import BoundingBoxesWithRealCoordinates
from rcl_interfaces.msg import SetParametersResult

import message_filters


from scipy.spatial.transform import Rotation


import numpy as np
import math



class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        queue_size = 500 #number of messages to be kept in queue
        synchronization_threshold = 1 #time [in seconds] allowed between message time stamps

        self.boundingBox_subscriber = message_filters.Subscriber(self, BoundingBoxes, '/bboxes')
        self.lidar_subscriber = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        self.odom_subscriber = message_filters.Subscriber(self, Odometry, '/odom')
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.boundingBox_subscriber,
                                                                         self.lidar_subscriber,
                                                                         self.odom_subscriber],
                                                                        queue_size = queue_size,
                                                                        slop = synchronization_threshold)
        self.synchronizer.registerCallback(self.callback_synchronized)

        self.bboxWithRealCoords_publisher = self.create_publisher(BoundingBoxesWithRealCoordinates, '/bboxes_realCoords', 10)

        self.classes = ['blue', 'orange', 'yellow']

        self.current_angle = 0

        print("Sensor Fusion Node started!")

    def callback_synchronized(self, bbox_msg, lidar_msg, odom_msg):
        self.get_logger().info('Receiving synchronized bboxes, lidar and odometry')
        self.sensor_fusion(bbox_msg, lidar_msg, odom_msg)


    def sensor_fusion(self, bbox_msg, lidar_msg, odom_msg):
        clustered_lidar = self.clusterLidarPoints(lidar_msg)
        bboxes = bbox_msg.bboxes
        #print("Clustered points: ", clustered_lidar)

        self.current_angle = Rotation.from_quat([odom_msg.pose.pose.orientation.x,
                                                 odom_msg.pose.pose.orientation.y,
                                                 odom_msg.pose.pose.orientation.z,
                                                 odom_msg.pose.pose.orientation.w]).as_euler('xyz', degrees=False)[2]

        """if odom_msg.pose.pose.orientation.x > 0:
            self.current_angle = round(math.acos(odom_msg.pose.pose.orientation.w) * 180 / math.pi * 2, 1)
        elif odom_msg.pose.pose.orientation.x < 0:
            self.current_angle = round(-math.acos(odom_msg.pose.pose.orientation.w) * 180 / math.pi * 2, 1)"""

        print(f"Current angle: {self.current_angle}")

        matchedBBoxes = []
        for bbox in bboxes:
            bestMatch = None
            for clusterAngle, clusterDistance in clustered_lidar:
                if bestMatch is None:
                    bestMatch = (clusterAngle, clusterDistance)
                elif self.distanceToBoxCenter(bbox, clusterAngle) < self.distanceToBoxCenter(bbox, bestMatch[0]):
                    bestMatch = (clusterAngle, clusterDistance)

            if not bestMatch is None:
                #print("bestMatch: ", bestMatch)
                #print("(boxleft, boxright, bestMatchPixel): ", bbox.coordinates[0], bbox.coordinates[2], self.angleToPixel(bestMatch[0]))
                odom_pos = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
                bboxPos = self.polarToCartesianMirrored(bestMatch[0] + 59 + self.current_angle, bestMatch[1])
                bboxPos[0] -= odom_pos[1]
                bboxPos[1] += odom_pos[0]
                print("BboxPos: ", bboxPos)
                print("BboxAngle: ", bestMatch[0] + 59 + (self.current_angle/np.pi)*180)
                print("OdomPos: ", odom_pos)
                bboxMsg = BoundingBoxWithRealCoordinates()
                bboxMsg.image_coords = bbox.coordinates
                bboxMsg.conf = bbox.conf
                bboxMsg.cls = bbox.cls
                bboxMsg.real_coords = bboxPos.tolist()


                matchedBBoxes.append(bboxMsg)

        BBoxesMsg = BoundingBoxesWithRealCoordinates()
        BBoxesMsg.header = Header()
        BBoxesMsg.header.stamp = self.get_clock().now().to_msg()
        BBoxesMsg.bboxes = matchedBBoxes
        self.bboxWithRealCoords_publisher.publish(BBoxesMsg)

        self.get_logger().info('Publishing bounding boxes with real coordinates')

    def angleToPixel(self, angle):
        return  640 - ((angle / 62.0) * 640)
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
        x = np.cos(angle) * distance
        y = np.sin(angle) * distance
        return np.array([x,y])


def main(args=None):
    rclpy.init(args=args)

    node = SensorFusionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()