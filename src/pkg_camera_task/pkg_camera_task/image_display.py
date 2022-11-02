import cv2 as cv
import rclpy
import datetime

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread


class ImageDisplay(Node):

    def __init__(self):

        super().__init__('sub')
        self.bridge = CvBridge()
        self.cv = cv
        self.create_subscription(Image, 'camera_feed', self.callback, 10)
        self.create_subscription(Image, 'camera_snap', self.save_snap, 10)

        self.controls_pub = self.create_publisher(String, 'controls', 10)

    def callback(self, msg):

        import pdb;pdb.set_trace()

        frame = self.bridge.imgmsg_to_cv2(msg)
        self.cv.imshow("preview", frame)
        #self.get_logger().info('-> %s' % "frame_received")
        cv.waitKey(1)

    def save_snap(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError:
            print(CvBridgeError)
        else:
            # Save your OpenCV2 image as a jpeg
            cv.imwrite(str(datetime.datetime.now()) + '_snap.jpeg', cv2_img)
            self.get_logger().info('-> %s' % "snap saved")

    def send_user_input(self, msg):
        print("tewtt " + msg)
        msg_obj = String()
        msg_obj.data = str(msg)
        print('command published: ' + msg_obj.data)
        self.controls_pub.publish(msg_obj)



def user_input(node):
    while True:
        command = input("Awaiting user input ('snap') or integer: ")
        if command == "snap" or command.isnumeric():
            node.send_user_input(str(command))
            print('Command send-> %s' % command)

def main(args=None):
    rclpy.init(args=args)
    sub = ImageDisplay()
    t = Thread(target=user_input, args=(sub,))
    t.start()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
