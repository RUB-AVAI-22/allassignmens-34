import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from avai_messages.msg import BoundingBoxWithRealCoordinates
from avai_messages.msg import BoundingBoxesWithRealCoordinates
from avai_messages.msg import Map
from avai_messages.msg import MapEntry
from geometry_msgs.msg import _pose_with_covariance
from geometry_msgs.msg import _twist_with_covariance
from geometry_msgs.msg import Vector3

from sklearn.cluster import DBSCAN

import numpy as np
import tf2_ros


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.classes = ['blue', 'orange', 'yellow']
        self.map_current = []  # entries denote objects in our map, each object consists of xy coordinates and a corresponding class
        self.position = [0.0, 0.0]
        self.heading_direction = 0.0 #from 0 to 2pi

        self.subscriber_bboxes_with_real_coordinates = self.create_subscription(BoundingBoxesWithRealCoordinates,
                                                                             '/bboxes_realCoords', self.callback_bbox,
                                                                                10)
        self.subscriber_odometry = self.create_subscription(Odometry, '/odom', self.callback_odometry, 10)

        self.publisher_map = self.create_publisher(Map, '/map', 10)

        self.msgs_bboxes = []
        self.msgs_odometry = []

        self.last_used_bboxes = None
        self.last_used_odometry = None

        self.timer_message_cleanup = self.create_timer(1, self.remove_old_messages)
        self.timer_map_update_attempt = self.create_timer(0.1, self.attempt_map_update)
        print("Mapping Node started!")

    def callback_bbox(self, msg):
        self.get_logger().info('Receiving bboxes')
        self.msgs_bboxes = np.append(self.msgs_bboxes, msg)

    def callback_odometry(self, msg):
        self.get_logger().info('Receiving odometry')
        self.msgs_odometry = np.append(self.msgs_odometry, msg)

    # Since we store all messages received we need to clean the array up to prevent infinite growth of memory usage
    # Every message older than 1 second is discarded
    def remove_old_messages(self):
        stamp_current = self.get_clock().now().to_msg()
        self.msgs_bboxes = np.array([msg_bbox for msg_bbox in self.msgs_bboxes if
                                     self.message_distance(msg_bbox.header.stamp, stamp_current) < 1])
        self.msgs_odometry = np.array([msg_odom for msg_odom in self.msgs_odometry if
                                       self.message_distance(msg_odom.header.stamp, stamp_current) < 1])

    # Each messages has a header with a timestamp
    # Each timestamp contains seconds and nanoseconds information
    # This function computes the distance between the two timestamps in seconds
    def message_distance(self, timestampA, timestampB):
        totalTimeA = timestampA.sec + timestampA.nanosec * 10e-9
        totalTimeB = timestampB.sec + timestampB.nanosec * 10e-9
        totalDistance = totalTimeB - totalTimeA
        return totalDistance

    # Since messages are transmitted over network, there can be delays and also lost packages
    # For the newest bounding box information we look for the closest odometry data.
    # If no match is found the map is not updated.
    def attempt_map_update(self):
        msg_selected_bboxes = None
        for msg_bboxes in np.flip(self.msgs_bboxes):
            closest_odometry_msg = None
            closest_distance = 0.1
            for msg_odometry in np.flip(self.msgs_odometry):
                msg_distance = self.message_distance(msg_bboxes.header.stamp, msg_odometry.header.stamp)
                if msg_distance < closest_distance:
                    closest_odometry_msg = msg_odometry
                    msg_selected_bboxes = msg_bboxes
                    break
            if closest_odometry_msg:
                self.update_map(msg_selected_bboxes, closest_odometry_msg)
                self.last_used_bboxes = msg_selected_bboxes
                self.last_used_odometry = closest_odometry_msg
                np.delete(self.msgs_odometry, np.where(self.msgs_odometry == closest_odometry_msg))
                np.delete(self.msgs_bboxes, np.where(self.msgs_bboxes == msg_selected_bboxes))
                break

    def update_map(self, msg_bboxes, msg_odometry):
        map_new = self.extract_xy_and_cls(msg_bboxes)

        map_odometry_integration = self.integrate_odometry(map_new, msg_odometry)

        # Maps are merged every iteration
        # Points are grouped depending on a minimum distance
        # Their positions are averaged and the result is added to the new map
        map_merged = np.empty((len(map_odometry_integration), 3))
        for i, newObject in enumerate(map_odometry_integration):
            closest_object = None
            closest_object_distance = 0.1
            for currentObject in self.map_current:
                dist_tmp = self.mapDistance(newObject, currentObject)
                if newObject[2] == currentObject[2] and dist_tmp < closest_object_distance:
                    closest_object = currentObject
                    closest_object_distance = dist_tmp
            if closest_object:
                map_merged[i] = self.merge_objects(newObject, closest_object)
            else:
                map_merged[i] = newObject

        # Cluster map using density clustering
        # The algorithm starts at a random point.
        # If there are at least min_samples points in a radius of eps around the point they are grouped into a cluster.
        # Each point in the cluster then searches around itself for more points to add into the cluster.
        # This again requires min_samples points in a radius of eps.
        clusterer = DBSCAN(eps=1, min_samples=5)
        map_clustered = np.zeros((map_merged.shape[0], 3))
        # Cluster only points in map belonging to certain class, since clusters only make sense with same class points
        for cls in range(len(self.classes)):
            indices_cls_subset = np.where(map_merged[:, 2] == cls)[0]

            map_cls_subset = map_merged[indices_cls_subset][:, :2]

            if len(map_cls_subset) == 0:
                break

            cluster_labels = clusterer.fit_predict(map_cls_subset)

            # Compute mean position of all points in a cluster and place as new point in map
            for i in range(max(cluster_labels)):
                indices = np.where(cluster_labels == i)[0]

                cluster_center = np.mean(map_cls_subset[indices])

                map_clustered[indices_cls_subset[indices]] = [*cluster_center, cls]

            # Take over the remaining points not belonging to any cluster
            indices = np.where(cluster_labels == -1)[0]
            for index in indices:
                map_clustered[indices_cls_subset[index]] = map_cls_subset[index]

        map_clustered = np.unique(map_clustered, axis=0)

        self.map_current = map_merged
        self.get_logger().info('Map updated!')
        self.publish_map(self.map_current)



    # Map array is converted to message objects and published
    def publish_map(self, map_current):
        msg_map = Map()
        map_objects = []
        for object in map_current:
            msg_map_object = MapEntry()
            msg_map_object.coordinates = object[:2].astype(np.float32)
            msg_map_object.cls = int(object[2])
            map_objects.append(msg_map_object)
        msg_map.map_objects = map_objects
        self.publisher_map.publish(msg_map)

    # Merging of two map objects requires averaging their position
    # Their class has to be the same for this to make sense
    def merge_objects(self, object_a, object_b):
        if not object_a[2] == object_b[2]:
            return
        object_merged = [0, 0, 0]
        object_merged[0] = (object_a[0] + object_b[0]) / 2
        object_merged[1] = (object_a[1] + object_b[1]) / 2
        object_merged[2] = object_a[2]
        return object_merged

    # The received bounding box messages contain information that is not relevant to us
    # Here we extract the relevant real world position and class information and compile it into the map format
    def extract_xy_and_cls(self, msg_bboxes):
        map_objects = np.empty((len(msg_bboxes.bboxes), 3))
        for i, bbox_msg in enumerate(msg_bboxes.bboxes):
            map_object = np.empty(3)
            map_object[0] = bbox_msg.real_coords[0]
            map_object[1] = bbox_msg.real_coords[1]
            map_object[2] = bbox_msg.cls
            map_objects[i] = map_object
        return map_objects

    # When the robot moves the map is supposed to change accordingly
    # For this we need to integrate our knowledge about the robots position into the extracted map
    # The odometry message type contains an integrated position vector which can be used for this
    def integrate_odometry(self, map_without_odometry, msg_odometry):
        """if self.last_used_odometry and self.last_used_bboxes:
            orientation_euler_previous = tf2_ros.transformations.euler_from_quaternion(self.last_used_odometry.pose.orientation)
            velocity_previous = self.last_used_odometry.twist.linear
            orientation_euler_current = tf2_ros.transformations.euler_from_quaternion(msg_odometry.pose.orientation)
            velocity_current = msg_odometry.twist.linear

            orientation_averaged = [(orientation_euler_previous[0] + orientation_euler_current[0]) / 2,
                                    (orientation_euler_previous[1] + orientation_euler_current[1]) / 2,
                                    (orientation_euler_previous[2] + orientation_euler_current[2]) / 2]

            heading_direction = orientation_averaged[2]

            velocity_averaged = [(velocity_previous.x + velocity_current.x) / 2,
                                 (velocity_previous.y + velocity_current.y) / 2,
                                 (velocity_previous.z + velocity_current.z) / 2]

            velocity_forward = velocity_averaged.x

            time_between_messages = self.message_distance(msg_odometry.header.stamp, self.last_used_odometry.header.stamp)

            distance_traveled = velocity_forward * time_between_messages

            translation = self.polarToCartesian(heading_direction, distance_traveled)

            self.position += translation

            position_current = msg_odometry.pose.position

            map_integrated = np.array([entry + msg_odometry for entry in map_without_odometry])

            return map_integrated
        else:
            return map_without_odometry"""

        position_point = msg_odometry.pose.pose.position

        position_current = np.array([position_point.x, position_point.y])


        map_integrated = map_without_odometry.copy()

        if len(map_integrated) == 0:
            return map_integrated

        map_integrated[:][0] += position_current[0]
        map_integrated[:][1] += position_current[1]

        return map_integrated

    def polarToCartesian(self, angle, distance):
        angle = (angle/180.0)*np.pi
        x = np.cos(angle) * distance
        y = np.sin(angle) * distance
        return [x,y]

    def mapDistance(self, object_a, objectB):
        coords_a = object_a[:2]
        coords_b = objectB[:2]
        return np.sqrt((coords_b[0] - coords_a[0]) ** 2 + (coords_b[1] - coords_a[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)

    node = MappingNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
