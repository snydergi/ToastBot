import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
import numpy as np

# Hardcode the transformation from Robot's base frame to the april tag
# Using the Realsense camera, locate the april tag that is mounted to the robot's base
# Now that we Know RobotBase -> BaseTag and Camera -> BaseTag,
# We need to manually publish the tf transform from RobotBase -> BaseTag


class Localizer(Node):
    pass
