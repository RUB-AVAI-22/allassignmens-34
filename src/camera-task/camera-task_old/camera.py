import rclpy
import cv2
import cv_bridge
from rclpy.node import Node
from std_msgs.msg import String



class Pub(Node):

    def __init__(self):
        super().__init__('pub')
        self.ctr = 0
        self.publisher_ = self.create_publisher(String, 'demo', 10)
        self.timer = self.create_timer(0.25, self.init_webcam())

    def init_webcam(self):
        import cv2

        # define a video capture object
        vid = cv2.VideoCapture("rtsp://web.nidaku.de:8554/avai")

        while (True):

            # Capture the video frame
            # by frame
            ret, frame = vid.read()

            # Display the resulting frame
            cv2.imshow('frame', frame)

            # the 'q' button is set as the
            # quitting button you may use any
            # desired button of your choice
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # After the loop release the cap object
        vid.release()
        # Destroy all the windows
        cv2.destroyAllWindows()

    def callback(self):
        self.ctr = self.ctr + 1
        msg = String()
        msg.data = "%d" % self.ctr
        self.publisher_.publish(msg)
        self.get_logger().info('<- %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    pub = Pub()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
