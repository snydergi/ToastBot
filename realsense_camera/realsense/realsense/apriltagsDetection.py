import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
import numpy as np

class ApriltagsNode(Node):
    def __init__(self):
        super().__init__("apriltags_subscriber")
        self.publisher = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.listener_callback, 10)
        self.detection_subscription = self.create_subscription(AprilTagDetectionArray, 'detections', self.detection_callback, 10)
        self.logger = self.get_logger().info

        self.target_detection = False
        self.target_centre = None
        self.bridge = CvBridge()

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            detection = msg.detections[0]
            self.get_logger().info(f'Tag ID: {detection.id}')
            self.get_logger().info(f'Centre: x={detection.centre.x}, y={detection.centre.y}')
            # self.get_logger().info(f'Pose: {detection.pose}')
            
            self.target_detection = True
            self.target_centre = (
                int(detection.centre.x), 
                int(detection.centre.y)
            )
        else:
            self.target_detection = False
            self.target_centre = None

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # Draw circle if target is detected
        if self.target_detection and self.target_centre is not None:
            cv2.circle(current_frame, self.target_centre, 10, (0, 255, 0), -1)  # Green filled circle
        
        # Display the camera feed
        cv2.imshow("Camera Feed with AprilTag", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ApriltagsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
