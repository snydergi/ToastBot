import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

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
    def __init__(self):
        super().__init__('localizer')
        self.static_broadcaster = StaticTransformBroadcaster(self)

    def robot_base_to_base_tag_transform(self) -> TransformStamped:
        # Return the Transform from RobotBase -> BaseTag
        robot_base_to_base_tag = TransformStamped()
        robot_base_to_base_tag.header.stamp = self.get_clock().now().to_msg()
        robot_base_to_base_tag.header.frame_id = "base_link"
