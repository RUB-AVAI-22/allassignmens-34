import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import SetParametersResult

import datetime
import os
import cv2


class ImgDisplayNode(Node):

    def __init__(self):
        super().__init__('image_display_node')
        # timer
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.control_callback)
        # video subscriber
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(CompressedImage, '/proc_img', self.video_callback, 10)

        # set parameters
        self.param_store_imgs = self.declare_parameter('store_imgs', False)
        self.param_imgs_path = self.declare_parameter('imgs_path', 'imgs')

        self.add_on_set_parameters_callback(self.parameter_callback)
        # self.publisher_ = self.create_publisher(String, 'FPS', 10)

        # save video
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter('out.mp4', fourcc, 20.0, (960, 480), True)

    def video_callback(self, frame):
        if frame:
            current_frame = self.bridge.compressed_imgmsg_to_cv2(frame)
            current_frame = cv2.resize(current_frame, (960, 480))
            cv2.imshow('frame', current_frame)
            cv2.waitKey(1)

        if self.param_store_imgs:
            if not os.path.exists(self.param_imgs_path.value):
                os.makedirs(self.param_imgs_path.value)
            cv2.imwrite(os.path.join(self.param_imgs_path.value, str(datetime.datetime.now()).replace(' ', '_')+'.jpeg') , current_frame)

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'store_imgs':
                self.get_logger().info(f'changed store_imgs from {self.param_store_imgs} to {parameter.value}')
                self.param_store_imgs = parameter.value
                return SetParametersResult(successful=True)
            if parameter.name == 'imgs_path':
                self.get_logger().info(f'changed store_imgs from {self.param_imgs_path} to {parameter.value}')
                self.param_imgs_path = parameter.value
                return SetParametersResult(successful=True)


def main(args=None):
    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

    rclpy.init(args=args)

    node = ImgDisplayNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
