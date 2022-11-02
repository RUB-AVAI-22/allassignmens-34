import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
import numpy as np

class Sub(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        self.subscriber = self.create_subscription(Image, 'video_frames', self.video_callback, 10)
        self.bridge = CvBridge()

        self.fps = 10
        self.declare_parameter("fps", int(self.fps))
        self.publisher_ = self.create_publisher(String, 'demo', 10)

        self.timer = self.create_timer(0.25, self.control_callback)
        fourcc = cv2. VideoWriter_fourcc(*'MP4V')
        self.out = cv2.VideoWriter('out.mp4', fourcc, 20.0, (640, 480))

    def control_callback(self):
        param = self.get_parameter("fps").get_parameter_value().integer_value

        msg = String()
        msg.data = "%d" % self.fps
        self.publisher_.publish(msg)
        self.get_logger().info('frame rate <- %s' % msg.data)

    def video_callback(self, frame):

        self.fps = cv2.getTrackbarPos('frame rate', 'frame')
        if self.fps == 0:
            self.fps = 1
        current_frame = self.bridge.imgmsg_to_cv2(frame)

        cv2.imshow('frame', current_frame)
        #self.get_logger().info('Receiving video frame')
        if cv2.waitKey(1) & 0xFF == ord('s'):
            self.get_logger().info('start saving video')
            cv2.imwrite('test.png',current_frame)
            self.out.write(current_frame)
        if cv2.waitKey(1) & 0xFF == ord('w'):
            self.get_logger().info('stopped')
            self.out.release()


def nothing(x):
    pass

def main(args=None):

    cv2.namedWindow('frame')
    cv2.createTrackbar('frame rate', 'frame', 10, 60, nothing)


    rclpy.init(args=args)

    sub = Sub()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
