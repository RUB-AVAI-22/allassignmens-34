import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

import datetime


class ImgProcessingNode(Node):

    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
        # subscriber for raw img data
        self.subscriber_img = self.create_subscription(Image, '/raw_image', self.callback, 10)
        # publisher for compressed img data
        self.publisher_ = self.create_publisher(CompressedImage, '/proc_img', 10)

    def callback(self, msg):
        print(f"received new image {datetime.datetime.now()}")

        img = self.bridge.imgmsg_to_cv2(msg)

        # convert image to compressed image
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(img)
        # publish compressed image
        self.publisher_.publish(compressed_image)


def main(args=None):
    rclpy.init(args=args)
    node = ImgProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
