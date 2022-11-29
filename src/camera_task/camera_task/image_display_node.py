import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import SetParametersResult

import datetime
import os

import queue


class ImgDisplayNode(Node):

    def __init__(self):
        super().__init__('image_display_node')
        # timer
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.control_callback)
        # video subscriber
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(CompressedImage, '/proc_img', self.video_callback, 10)
        self.bbox_subscriber = self.create_subscription(Float32MultiArray, '\bounding_boxes', self.bbox_callback, 10)

        self.bbox_queue = queue.SimpleQueue()
        self.image_queue = queue.SimpleQueue()

        self.param_fps = self.get_parameter('FPS').value
        self.timer = self.create_timer(1 / self.param_fps, self.annotation)


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
            image_queue.put(current_frame)

        if self.param_store_imgs:
            if not os.path.exists(self.param_imgs_path.value):
                os.makedirs(self.param_imgs_path.value)
            cv2.imwrite(os.path.join(self.param_imgs_path.value, str(datetime.datetime.now()).replace(' ', '_')+'.jpeg') , current_frame)
            self.param_store_imgs = False
    def bbox_callback(self, msg):
        bboxes = msg.data

        self.bbox_queue.put(bboxes)

    def annotation(self):
        if self.bbox_queue.qsize() >= 1 and self.image_queue.qsize() >= 1:
            image = self.image_queue.get()
            bboxes = self.bbox_queue.get()


            # annotating image with detected bounding boxes
            annotator = Annotator(image)
            for *xyxy, conf, cls in pred[0]:
                if conf > 0.7:
                    annotator.box_label(xyxy, f'{self.classes[int(cls)]} {conf:.2f}')

            annotated_image = annotator.result()


            annotated_image = cv2.resize(annotated_image, (960, 480))
            cv2.imshow('frame', current_frame)
            cv2.waitKey(1)

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
