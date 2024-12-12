"""
Contains publishers and TF listener to update April Tag poses.

PUBLISHERS:
  + '/toast/loafTrayPose' (geometry_msgs/msg/Pose) - Pose of the loaf tray April Tag
  + '/toast/leverPose' (geometry_msgs/msg/Pose) - Pose of the toaster lever April Tag
  + '/toast/slidePose' (geometry_msgs/msg/Pose) - Pose of the slide April Tag
  + '/toast/brushPose' (geometry_msgs/msg/Pose) - Pose of the brush April Tag
  + '/toast/bowlPose' (geometry_msgs/msg/Pose) - Pose of the butter bowl April Tag
"""

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, Transform


class TransformAuditor(Node):
    """TODO."""

    def __init__(self):
        """Initialize node."""
        super().__init__('transform_auditor')
        self.get_logger().debug('Transform Auditor Started!')
        pubQoS = 10
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.loaf_tray_pose_pub = self.create_publisher(
            Pose, '/toast/loafTrayPose', pubQoS
        )
        self.lever_pose_pub = self.create_publisher(
            Pose, '/toast/leverPose', pubQoS
        )
        self.brush_pose_pub = self.create_publisher(
            Pose, '/toast/brushPose', pubQoS
        )
        self.bowl_pose_pub = self.create_publisher(
            Pose, '/toast/bowlPose', pubQoS
        )
        self.slide_pose_pub = self.create_publisher(
            Pose, '/toast/slidePose', pubQoS
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Request transforms and publish if not none."""
        loaf_tray_tf = self.get_transform('loaf_tray')
        lever_tf = self.get_transform('lever')
        brush_tf = self.get_transform('brush')
        bowl_tf = self.get_transform('bowl')
        slide_tf = self.get_transform('slide')
        if loaf_tray_tf:
            self.loaf_tray_pose_pub.publish(
                self.transform_to_pose(loaf_tray_tf))
        if lever_tf:
            self.lever_pose_pub.publish(self.transform_to_pose(lever_tf))
        if brush_tf:
            self.brush_pose_pub.publish(self.transform_to_pose(brush_tf))
        if bowl_tf:
            self.bowl_pose_pub.publish(self.transform_to_pose(bowl_tf))
        if slide_tf:
            self.slide_pose_pub.publish(self.transform_to_pose(slide_tf))

    def transform_to_pose(self, tf: Transform) -> Pose:
        """
        Publish pose of given transform.

        :param tf: Transform from base to previously specified frame.
        :type tf: Transform
        :return pose: Pose representing child frame origin
        :rtype pose: Pose
        """
        pose = Pose()
        pose.position.x = tf.translation.x
        pose.position.y = tf.translation.y
        pose.position.z = tf.translation.z
        pose.orientation = tf.rotation
        return pose

    def get_transform(self, targetFrameID: str) -> Transform:
        """
        Get transform from target frame to base frame.

        :param targetFrameID: Desired frame ID
        :type targetFrameID: str
        :return: Transformation from robot base to target frame
        :rtype: Transform
        """
        try:
            tf = self.buffer.lookup_transform(
                source_frame=targetFrameID,
                target_frame='base',
                time=rclpy.time.Time()
            )
            return tf.transform
        except Exception as e:
            self.get_logger().debug(
                f'Failed transform to {targetFrameID} with exception: {e}'
            )
        return None


def main(args=None):
    """Run node."""
    rclpy.init(args=args)
    transform_auditor = TransformAuditor()
    rclpy.spin(transform_auditor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
