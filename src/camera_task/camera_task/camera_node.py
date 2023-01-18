import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult

import numpy as np


class CameraNode(Node):

    def __init__(self, use_webcam):
        super().__init__('camera_node')
        # publisher for raw img data
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # camera stream
        if use_webcam:
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = cv2.VideoCapture('rtsp://web.nidaku.de:8554/avai')
        self.bridge = CvBridge()

        # ros parameters
        self.declare_parameter('FPS', 10)
        self.param_fps = self.get_parameter('FPS').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # timer
        self.timer = self.create_timer(1 / self.param_fps, self.video_callback)

        print("Node started!")

    def video_callback(self):
        ret, frame = self.cap.read()

        if ret:
            msg = self.bridge.cv2_to_imgmsg(np.array(frame))

            self.publisher_.publish(msg)

            self.get_logger().info('Publishing video frame')

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == "FPS":
                self.get_logger().info('---PARAM CHANGE--- frame rate <- %s' % parameter.value)
                fps = parameter.value
                freq = 1 / fps

                tmp = self.timer

                self.timer = self.create_timer(freq, self.video_callback)
                tmp.cancel()
                return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = CameraNode(True)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
