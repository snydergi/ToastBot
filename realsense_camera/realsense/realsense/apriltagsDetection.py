"""
AprilTags Detection and Visualization Node

This script defines a ROS 2 node for detecting AprilTags in a live camera feed, drawing 
visual markers on the detected tags, and providing relevant logging information. It uses 
the `sensor_msgs/Image` topic for camera data and the `apriltag_msgs/AprilTagDetectionArray` 
topic for AprilTag detection messages.

Classes:
--------
- `ApriltagsNode`: A custom ROS 2 node that subscribes to the camera feed and AprilTag 
  detection topics, processes the data, and displays the live feed with visual markers.

Functions:
----------
- `detection_callback(msg)`: Handles AprilTag detection messages, extracts information 
  about the first detected tag (e.g., ID, center coordinates), and updates the target 
  detection status.
- `listener_callback(data)`: Processes the live camera feed, overlays a visual marker 
  (circle) on the detected tag's position, and displays the feed using OpenCV.

Main Execution:
---------------
- Initializes the ROS 2 node.
- Subscribes to:
  - `/camera/camera/color/image_raw`: The camera's RGB image topic.
  - `detections`: The AprilTag detection topic.
- Publishes:
  - `/camera/camera/color/image_raw` (reserved for future extensions if needed).
- Spins the node until shutdown, allowing continuous tag detection and visualization.

Dependencies:
-------------
- ROS 2
- OpenCV (for image processing and visualization)
- cv_bridge (for converting ROS image messages to OpenCV format)
- AprilTag ROS 2 package (`apriltag_msgs` for detection messages)

Usage:
------
Run the node as part of a ROS 2 system where a camera node (e.g., RealSense) is publishing 
RGB images and an AprilTag detection node is running. This node processes and visualizes 
the detections on the live feed.

Topics:
-------
- Subscribed:
  - `/camera/camera/color/image_raw`: RGB image stream.
  - `detections`: AprilTag detection data.
- Published:
  - `/camera/camera/color/image_raw`: (Future extensions, not utilized currently).
"""


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
            # self.get_logger().info(f'Tag ID: {detection.id}')
            # self.get_logger().info(f'Centre: x={detection.centre.x}, y={detection.centre.y}')
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
