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
# And then we need to then publish the tf transform from Camera -> BaseTag
# so we can connect the TF tree.
# This node should not publish to any topics but should publish the TF of the Camera
# to the BaseTag and all other tags it sees so other nodes can look up
# RobotBase -> <AnyTag> since


class Localizer(Node):
    pass
