"""TODO."""

import rclpy
from rclpy.node import Node
from moveitapi.mpi import MotionPlanningInterface
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose


class ToastBot(Node):
    """TODO."""

    def __init__(self):
        """TODO."""
        super().__init__('toastBot')
        self.get_logger().info('ToastBot Started!')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.mpi = MotionPlanningInterface(self)
        self.setScene = self.create_service(Empty, 'buildScene', self.setScene_callback,
                                            callback_group=client_cb_group)

    async def setScene_callback(self, request, response):
        """
        Set scene of demonstration.

        :param request: Request object
        :type request: std_srvs/srv/Empty
        :param response: Response object
        :type response: std_srvs/srv/Empty
        """
        self.get_logger().info('SetScene Callback called!')

        tableSectionOne = Pose()
        tableSectionOne.position.x = 0.0
        tableSectionOne.position.y = 1.0
        tableSectionOne.position.z = 0.0
        tableSectionOneDims = [0.80645, 1.2446, 0.0127]
        section1Shape = 'Box'

        tableSectionTwo = Pose()
        tableSectionTwo.position.x = 0.0
        tableSectionTwo.position.y = 0.0
        tableSectionTwo.position.z = 1.0
        tableSectionTwoDims = [0.4064, 0.508, 0.0127]
        section2Shape = 'Box'

        pedastle = Pose()
        pedastle.position.x = 1.0
        pedastle.position.y = 0.0
        pedastle.position.z = 0.0
        pedastleDims = [0.3048, 0.2159, 0.2159]
        pedastleShape = 'Box'

        await self.mpi.loadPlanningScene([tableSectionOne, tableSectionTwo, pedastle],
                                         [tableSectionOneDims, tableSectionTwoDims, pedastleDims],
                                         [section1Shape, section2Shape, pedastleShape])

        return response


def main(args=None):
    """Run node."""
    rclpy.init(args=args)
    toastBot = ToastBot()
    rclpy.spin(toastBot)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
