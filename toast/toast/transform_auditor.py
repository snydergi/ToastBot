"""TODO."""

import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, Transform


class TransformAuditor(Node):
    """TODO."""

    def __init__(self):
        """Initialize node."""
        super().__init__('transform_auditor')
        self.get_logger().debug('Transform Auditor Started!')
        # pubQoS = QoSProfile(
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        # )
        pubQoS = 10
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.loaf_tray_pose_pub = self.create_publisher(Pose, '/toast/loafTrayPose', pubQoS)
        self.lever_pose_pub = self.create_publisher(Pose, '/toast/leverPose', pubQoS)
        self.plate_pose_pub = self.create_publisher(Pose, '/toast/platePose', pubQoS)
        self.knife_pose_pub = self.create_publisher(Pose, '/toast/knifePose', pubQoS)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Request transforms and publish if not none."""
        loaf_tray_tf = self.get_transform('loaf_tray')
        lever_tf = self.get_transform('lever')
        plate_tf = self.get_transform('plate')
        knife_tf = self.get_transform('knife_holder')
        if loaf_tray_tf:
            self.loaf_tray_pose_pub.publish(self.transform_to_pose(loaf_tray_tf))
        if lever_tf:
            self.lever_pose_pub.publish(self.transform_to_pose(lever_tf))
        if plate_tf:
            self.plate_pose_pub.publish(self.transform_to_pose(plate_tf))
        if knife_tf:
            self.knife_pose_pub.publish(self.transform_to_pose(knife_tf))

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
                source_frame='base',
                target_frame=targetFrameID,
                time=rclpy.time.Time()
            )
            return tf.transform
        except Exception as e:
            self.get_logger().error(f'Failed transform to {targetFrameID} with exception: {e}')
        return None


def main(args=None):
    """Run node."""
    rclpy.init(args=args)
    transform_auditor = TransformAuditor()
    rclpy.spin(transform_auditor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
