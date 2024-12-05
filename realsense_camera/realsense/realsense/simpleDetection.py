"""
Simple Detection Node.

This module defines the `SimpleDetectionNode` class, designed to detect white objects.

Functions:
Detect Object by Color:
- Detects an object by a color, in this we are using white

Service and Action Integration:
- Integrates with MoveIt services and actions:
    - MoveGroup Action for trajectory execution.
    - Cartesian path computation service for straight-line trajectories.
- Handles feedback during action execution for real-time status updates.

Authors:
    Asa Rogers
    Date: 2024-12-05
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimpleDetectionNode(Node):
    """Class for Simple Detection Node."""

    def __init__(self):
        """Init detection node."""
        super().__init__('simple_detection_node')
        # CV Bridge for converting between ROS images and OpenCV images
        self.bridge = CvBridge()
        # Subscribers and Publishers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/detection/debug_image',
            10
        )
        # Define color ranges for detection (in HSV)
        self.color_ranges = {
            'white': [(0, 0, 200), (180, 30, 255)],   # White objects
            'yellow': [(20, 100, 100), (30, 255, 255)],  # Yellow objects
            'blue': [(100, 100, 100), (140, 255, 255)],  # Blue objects
        }
        self.get_logger().info('Simple Color-Based Detection Node initialized')

    def detect_objects_by_color(self, hsv_image):
        """Detect objects."""
        detections = {}
        for color_name, (lower, upper) in self.color_ranges.items():
            # Create a mask for the specific color range
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Filter and process contours
            valid_contours = []
            for contour in contours:
                # Filter by area to remove noise
                area = cv2.contourArea(contour)
                if area > 100:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    valid_contours.append((x, y, w, h))
            detections[color_name] = valid_contours
        return detections

    def image_callback(self, msg):
        """Check the image."""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Convert to HSV color space
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            # Detect objects by color
            detections = self.detect_objects_by_color(hsv_image)
            # Draw bounding boxes on debug image
            debug_image = cv_image.copy()
            for color, objects in detections.items():
                for (x, y, w, h) in objects:
                    # Draw rectangle
                    cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Put text
                    cv2.putText(debug_image,
                                f'{color.capitalize()} Object',
                                (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2)
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header = msg.header
            self.debug_image_publisher.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')


def main(args=None):
    """Start main function."""
    rclpy.init(args=args)
    node = SimpleDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
