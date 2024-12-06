"""
Example of a ROS 2 node for listening to the TF tree and extracting transformations.

This script demonstrates how to use the TF2 library to listen to transformations
in a simulation or real-world robotic system, specifically from a camera to various
objects in the environment.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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


class TfFrameListener(Node):
    """ROS 2 node for listening to TF transformations and extracting homogenous matrices."""

    def __init__(self):
        """Initialize the TF Frame Listener node."""
        super().__init__('tf_frame_listener')

        # Logger for informational messages
        self.logger = self.get_logger()

        # Declare and acquire the 'target_frame' parameter
        self.source_frame = self.declare_parameter(
            'camera_frame', 'camera_color_optical_frame').get_parameter_value().string_value

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cameraToRobotbase = TransformStamped()
        self.cameraToEnd_Effector = TransformStamped()
        self.cameraToToaster = TransformStamped()
        self.cameraToKnife_holder = TransformStamped()
        self.cameraToPlate = TransformStamped()

        # Create a timer to periodically fetch transformations
        self.timer = self.create_timer(1.0, self.update_transformations)

    def update_transformations(self):
        """Update the transformation matrices for various target frames."""
        self.camera_to_robot_base = self.get_homogeneous_matrix(
            target_frame='robot_base')
        self.camera_to_end_effector = self.get_homogeneous_matrix(
            target_frame='end_effector')
        self.camera_to_toaster = self.get_homogeneous_matrix(
            target_frame='toaster')
        self.camera_to_knife_holder = self.get_homogeneous_matrix(
            target_frame='knife_holder')
        self.camera_to_plate = self.get_homogeneous_matrix(
            target_frame='plate')

    def robot_base_to_base_tag_transform(self) -> TransformStamped:
        # Return the Transform from RobotBase -> BaseTag
        robot_base_to_base_tag = TransformStamped()
        robot_base_to_base_tag.header.stamp = self.get_clock().now().to_msg()
        robot_base_to_base_tag.header.frame_id = "base_link"

    def get_homogeneous_matrix(self, to_frame: str) -> TransformStamped:
        """
        Retrieve the homogeneous transformation matrix from the camera to the specified frame.

        Args:
            to_frame (str): The name of the target frame to transform to.

        Returns:
            TransformStamped: The transformation matrix as a TransformStamped message
        """
        from_frame = self.source_frame
        try:
            # Lookup transformation from target frame to camera frame
            transformation = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())
            return transformation
        except TransformException as ex:
            self.logger.info(
                f'Could not transform {from_frame} to {to_frame}: {ex}'
            )
            return None


def main():
    """Set up the node, spins it to handle callbacks, and gracefully shuts down."""
    rclpy.init()
    node = TfFrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()