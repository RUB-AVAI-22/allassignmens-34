import cv2 as cv
import rclpy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from std_msgs.msg import String


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('sub')
        self.bridge = CvBridge()

        # Listening to camera
        self.create_subscription(Image, 'raw_camera_feed', self.send_image, 10)
        self.create_subscription(Image, 'raw_camera_snap', self.send_snap, 10)
        # Publishing towards camera
        self.camera_actions_pub = self.create_publisher(String, 'camera_actions', 10)
        self.camera_settings_pub = self.create_publisher(String, 'camera_settings', 10)

        # Listening to image_display
        self.create_subscription(String, 'controls', self.controller, 10)
        # Publishing towards image_display
        self.camera_feed_pub = self.create_publisher(Image, 'camera_feed', 10)
        self.camera_snap_pub = self.create_publisher(Image, 'camera_snap', 10)

    def send_image(self, msg):

        self.camera_feed_pub.publish(msg)

    def send_snap(self, msg):
        print("snap received from camere, forwarding")
        self.camera_snap_pub.publish(msg)

    def controller(self, msg):
        print("command received")
        if msg.data == "snap":
            self.request_snap()
        elif msg.data.isnumeric():
            self.request_frequency_change(msg)

    def request_frequency_change(self, new_frequency):
        request = String()
        request.data = "frequency," + str(new_frequency.data) + ";"
        self.camera_settings_pub.publish(request)

    def request_snap(self):
        request = String()
        request.data = "1"
        self.camera_actions_pub.publish(request)


def main(args=None):
    rclpy.init(args=args)

    process_node = ImageProcessor()
    rclpy.spin(process_node)

    process_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
