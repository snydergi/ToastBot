"""TODO."""

import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener


class TransformAuditor(Node):
    """TODO."""

    def __init__(self):
        """Initialize node."""
        super().__init__('transform_auditor')
        self.get_logger().debug('Transform Auditor Started!')


def main(args=None):
    """Run node."""
    rclpy.init(args=args)
    transform_auditor = TransformAuditor()
    rclpy.spin(transform_auditor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
