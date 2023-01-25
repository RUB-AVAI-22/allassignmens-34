import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult

import numpy as np

import argparse

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

        print("Camera Node started!")

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

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Camera Node')
    parser.add_argument('-w', '--use_webcam', type=str2bool, default=True, help='Enable webcam')
    args = parser.parse_args()

    print(f'Camera {"enabled" if args.use_webcam else "disabled"}')

    node = CameraNode(args.use_webcam)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
