import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np


class Pub(Node):

    def __init__(self):
        super().__init__('camera')
        # publisher for raw img data
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        # canera stream
        self.cap = cv2.VideoCapture('rtsp://web.nidaku.de:8554/avai')
        self.bridge = CvBridge()

        # ros parameters
        self.freq = .2
        self.declare_parameter("fps", int(self.freq))
        self.add_on_set_parameters_callback(self.on_param_change)

        self.timer = self.create_timer(self.freq, self.video_callback)

        self.create_subscription(String, 'demo', self.on_param_change, 10)

    def on_param_change(self, parameters):
        #for parameter in parameters:
         #   if parameter.name == "fps":
        self.get_logger().info('frame rate <- %s' % parameters.data)
        fps = int(parameters.data)
        self.freq = 1 / fps


        tmp = self.timer

        self.timer = self.create_timer(self.freq, self.video_callback)
        tmp.cancel()
        #tmp.destroy()
                #return SetParametersResult(successful=True)

        # self.timer.reset()
        # self.timer = self.create_timer(self.freq, self.video_callback)
        # self.timer = self.create_timer(timer_period, self.video_callback)
        # print("---PARAM CHANGE---")
        # for parameter in parameters:
        # if parameter.name == "fps":
        #   fps =parameter.value
        #   print(f"chaged fps from {1 / self.freq} to {fps}")
        # reinitialize timer

        #  print("recreadted timer")
        #  return  Set

    def video_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(frame), "bgr8"))
        # self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)

    pub = Pub()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
