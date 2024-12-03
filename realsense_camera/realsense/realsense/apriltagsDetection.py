import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ApriltagsNode(Node):
    def __init__(self):
        super().__init__("apriltags_subscriber")
        self.publisher = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.listener_callback, 10)
        self.subscription

        self.bridge = CvBridge()

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

    
def main(args=None):
    rclpy.init(args=args)
    node = ApriltagsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()