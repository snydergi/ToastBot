"""TODO."""

from geometry_msgs.msg import Pose
from moveitapi.mpi import MotionPlanningInterface
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


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
        self.breadToToaster = self.create_service(Empty, 'breadToToaster', self.breadToToaster_callback,
                                                  callback_group=client_cb_group)
        self.breadNumber = 1 # So the franka picks the correct piece of bread
        # Transform Listener
        self.tfBuffer = Buffer()
        self.tfListeneristener = TransformListener(self.tfBuffer, self)
        

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
    
    async def breadToToaster_callback(self, request, response):
        """Move a piece of bread from the loaf holder to the toaster.
        
        This function moves the gripper to a piece of bread, grips the piece of bread, 
        moves the piece of bread to the toaster, then releasses the bread.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        self.get_logger().info("BreadToToaster Callback called!")
        if self.breadNumber > 4:
            self.get_logger().warn("All out of bread!")
        else:
            # Open the gripper
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)
            
            # Move the gripper to be above a slice of toast
            try:
                base_loaf_trans = self.buffer.lookup_transform(
                        'base_link', 'loaf_holder', rclpy.time.Time())  ########### Is this the correct transform?
            except tf2_ros.LookupException as e:
                    self.get_logger().info(f'Tranform lookup exception: {e}')
            except tf2_ros.ConnectivityException as e:
                    self.get_logger().info(f'Transform connectivity exception: {e}')
            except tf2_ros.ExtrapolationException as e:
                    self.get_logger().info(f'Transform extrapolation exception: {e}')
            
            #### Set theses value to match real world
            slice_offset_x = 0
            slice_offset_z = 0
            #### 
            current_pose = await self.mpi.getCurrentPose()
            goal = [
                base_loaf_trans.transform.translation.x + slice_offset_x * self.breadNumber,
                base_loaf_trans.transform.translation.y,
                base_loaf_trans.transform.translation.z + slice_offset_z,
                base_loaf_trans.transform.rotation.x,
                base_loaf_trans.transform.rotation.y,
                base_loaf_trans.transform.rotation.z,
                base_loaf_trans.transform.rotation.w
                ]
            pathType = 'POSE'
            await self.mpi.planPath(pathType, goal, execute=True)
            
            self.breadNumber += 1 # So franka knows which slice to grab
        return response


def main(args=None):
    """Run node."""
    rclpy.init(args=args)
    toastBot = ToastBot()
    rclpy.spin(toastBot)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
