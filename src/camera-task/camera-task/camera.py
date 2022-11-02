import cv2 as cv
import cv_bridge
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from std_msgs.msg import String


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.cap = None
        self.settings = {'frequency': 30}
        self.init_camera()

        # Listening to image_processing
        self.create_subscription(String, 'camera_actions', self.send_snap, 10)
        self.create_subscription(String, 'camera_settings', self.parse_settings, 10)
        # Publishing towards image_processing
        self.raw_camera_feed_pub = self.create_publisher(Image, 'raw_camera_feed', 10)
        self.raw_camera_snap_pub = self.create_publisher(Image, 'raw_camera_snap', 10)

        self.hz = 1 / self.settings['frequency']
        self.timer = self.create_timer(self.hz, self.send_image)

    def send_snap(self, count):
        print("sending snap..")
        snap = self.capture_image()
        self.raw_camera_snap_pub.publish(self.bridge.cv2_to_imgmsg(snap))

    def update_settings(self):
        print("Canceling Timer")
        self.timer.cancel()
        print("Frequency in settings: " + self.settings['frequency'])
        print("Applying frequency: " + str(self.hz))
        self.hz = 1 / int(self.settings['frequency'])
        self.timer = self.create_timer(self.hz, self.send_image)

    def parse_settings(self, msg):
        changes = msg.data.split(';')
        print("Parsed settings: ")
        print(changes)
        for x in changes:
            if x != "":
                print("Changes: ")
                print(x)
                option, value = x.split(',')
                self.change_settings(option, value)

    def change_settings(self, option, value):
        self.settings[option] = value
        print(self.settings)
        self.update_settings()


    def send_image(self):
        new_frame = self.capture_image()
        self.raw_camera_feed_pub.publish(self.bridge.cv2_to_imgmsg(new_frame))
        #self.get_logger().info('<- %s' % "image_sent")

    def init_camera(self):
        self.cap = cv.VideoCapture(0)
        if not self.cap.isOpened():
            IOError("Cannot open camera")

        cv.destroyAllWindows()

    def capture_image(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        # if frame is read correctly ret is True
        if not ret:
            IOError("Can't receive frame (stream end?). Exiting ...")
        return frame


def main(args=None):
    rclpy.init(args=args)
    camera = CameraNode()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
