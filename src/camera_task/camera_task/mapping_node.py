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
from nav_msgs.msg import Odometry

from sklearn.cluster import DBSCAN

import message_filters

import numpy as np
import tf2_ros


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.classes = ['blue', 'orange', 'yellow']
        self.map_current = []  # entries denote objects in our map, each object consists of xy coordinates and a corresponding class
        self.map_clustered = []
        self.current_pos = np.array([0.0, 0.0])

        self.subscriber_bboxes_with_real_coordinates = message_filters.Subscriber(self, BoundingBoxesWithRealCoordinates,
                                                                             '/bboxes_realCoords')
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.subscriber_bboxes_with_real_coordinates],
                                                                        100, 0.1)
        self.synchronizer.registerCallback(self.callback_synchronized)

        self.publisher_map = self.create_publisher(Map, '/map', 10)

        print("Mapping Node started!")

    def callback_synchronized(self, msg_bbox):
        self.get_logger().info('Receiving bboxes')
        self.update_map(msg_bbox)

    def update_map(self, msg_bbox):
        map_new = self.extract_xy_and_cls(msg_bbox)

        if len(self.map_current) == 0:
            map_merged = map_new
        else:
            map_merged = np.concatenate((map_new, self.map_current))

        # Cluster map using density clustering
        # The algorithm starts at a random point.
        # If there are at least min_samples points in a radius of eps around the point they are grouped into a cluster.
        # Each point in the cluster then searches around itself for more points to add into the cluster.
        # This again requires min_samples points in a radius of eps.
        map_clustered = np.array([])
        # Cluster only points in map belonging to certain class, since clusters only make sense with same class points
        for cls in range(len(self.classes)):
            clusterer = DBSCAN(eps=0.1, min_samples=8)
            indices_cls_subset = np.where(map_merged[:, 2] == cls)[0]

            map_cls_subset = map_merged[indices_cls_subset][:]
            #print(f"map_cls_subset {map_cls_subset}")
            if len(map_cls_subset) == 0:
                continue

            cluster_labels = clusterer.fit_predict(map_cls_subset)
            #print(f"cluster_labels {cluster_labels}")
            # Compute mean position of all points in a cluster and place as new point in map
            for i in range(max(cluster_labels)+1):
                indices = np.where(cluster_labels == i)[0]

                #print(f"indices {indices}")
                cluster_center_x = np.mean(map_cls_subset[indices, 0])
                cluster_center_y = np.mean(map_cls_subset[indices, 1])

                if len(map_clustered) == 0:
                    map_clustered = [[cluster_center_x, cluster_center_y, cls]]
                else:
                    map_clustered = np.concatenate((map_clustered, [[cluster_center_x, cluster_center_y, cls]]))

            # Take over the remaining points not belonging to any cluster
            #indices = np.where(cluster_labels == -1)[0]
            #print(indices)
            #map_clustered = np.delete(map_clustered, indices)
            #print(map_clustered)
            #for index in indices:
                #map_clustered[indices_cls_subset[index]] = map_cls_subset[index]

        #map_clustered = np.unique(map_clustered, axis=0)

        self.map_clustered = map_clustered
        self.map_current = map_merged
        #print(self.map_current)
        #print(self.map_clustered)
        self.get_logger().info('Map updated!')
        self.publish_map(self.map_clustered)



    # Map array is converted to message objects and published
    def publish_map(self, map_current):
        msg_map = Map()
        map_objects = []
        for object in map_current:
            object = np.array(object)
            msg_map_object = MapEntry()
            msg_map_object.coordinates = object[:2].astype(np.float32)
            msg_map_object.cls = int(object[2])
            map_objects.append(msg_map_object)
        msg_map.map_objects = map_objects
        self.publisher_map.publish(msg_map)
        self.get_logger().info("Published new Map!")

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
    def integrate_odometry(self, map_without_odometry, current_pos):

        map_integrated = map_without_odometry.copy()

        if len(map_integrated) == 0:
            return map_integrated

        #print(map_integrated.shape)
        #print(position_current.shape)


        map_integrated[:, 0] += current_pos[1]
        map_integrated[:, 1] += current_pos[0]

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
