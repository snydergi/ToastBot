import cv2
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import os
from ament_index_python.packages import get_package_prefix
import pyrealsense2 as rs
from ultralytics import YOLO
import numpy as np

class YOLOv8RealSense(Node):
    """
    This node runs YOLOv8 object detection on RealSense camera streams
    """

    def __init__(self):
        """
        Initializes the node, creates publishers and subscribers, and loads the YOLO model
        """
        super().__init__('yolov8_realsense')
        self.bridge = CvBridge()
        self.grayscale_sub = self.create_subscription(
            Image,
            '/camera/infra1/image_rect_raw',
            self.grayscale_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/infra1/camera_info',
            self.camera_info_callback,
            10)
        
        self.frequency = 200
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)

        # Load pre-trained YOLO model
        self.model = YOLO('yolov8n.pt').to('cuda' if torch.cuda.is_available() else 'cpu')

        self.grayscale_image = None
        self.depth_image = None
        self.intrinsics = None

    def camera_info_callback(self, msg):
        """
        Callback function for the subscriber that subscribes to /camera/infra1/camera_info
        
        Args: msg: Message containing camera metadata and calibration info

        Returns: None
        """
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = msg.height
        self.intrinsics.height = msg.width
        self.intrinsics.ppx = msg.k[5]
        self.intrinsics.ppy = msg.width - msg.k[2]
        self.intrinsics.fx = msg.k[4]
        self.intrinsics.fy = msg.k[0]

        if msg.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intrinsics.model = rs.distortion.kannala_brandt4

        self.intrinsics.coeffs = [i for i in msg.d]

    def grayscale_callback(self, msg):
        """
        Callback function for the subscriber to /camera/infra1/image_rect_raw
        Processes and stores the incoming grayscale image.

        Args: msg: Incoming message containing the grayscale image

        Returns: None
        """
        grayscale_image = self.bridge.imgmsg_to_cv2(msg)
        self.grayscale_image = cv2.rotate(grayscale_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.grayscale_image = cv2.cvtColor(self.grayscale_image, cv2.COLOR_GRAY2BGR)

    def depth_callback(self, msg):
        """
        Callback function for the subscriber to /camera/depth/image_rect_raw
        Processes and stores the incoming depth image.

        Args: msg: Incoming message containing the depth image

        Returns: None
        """
        depth_image = self.bridge.imgmsg_to_cv2(msg)
        self.depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def timer_callback(self):
        """
        Timer callback to perform object detection on the grayscale image.

        Returns: None
        """
        if self.grayscale_image is not None:
            results = self.model(self.grayscale_image, stream=False)
            annotated_image = results[0].plot()
            cv2.imshow('YOLOv8 Detection', annotated_image)
            cv2.waitKey(1)

def main(args=None):
    """
    The main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = YOLOv8RealSense()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
