"""TODO."""

from geometry_msgs.msg import Pose
from moveitapi.mpi import MotionPlanningInterface
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
import math


class ToastBot(Node):
    """TODO."""

    def __init__(self):
        """TODO."""
        super().__init__('toast_bot')
        self.get_logger().info('ToastBot Started!')
        # Links 1-7 Joint positions for home position [deg]
        self.home_joints = [0, -45, 0, -135, 0, 90, 45]
        # convert to radians
        self.home_joints = [math.radians(i) for i in self.home_joints]
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.mpi = MotionPlanningInterface(self)
        self.setScene = self.create_service(Empty, 'buildScene', self.setScene_callback,
                                            callback_group=client_cb_group)
        self.breadToToaster = self.create_service(
            Empty, 'breadToToaster', self.breadToToaster_callback, callback_group=client_cb_group
        )
        self.breadNumber = 1  # So the franka picks the correct piece of bread
        self.loaf_tray_pose_sub = self.create_subscription(
            Pose, '/toast/loafTrayPose', self.loaf_tray_pose_sub_cb, 10
        )
        self.lever_pose_sub = self.create_subscription(
            Pose, '/toast/leverPose', self.lever_pose_sub_cb, 10
        )
        self.plate_pose_sub = self.create_subscription(
            Pose, '/toast/platePose', self.plate_pose_sub_cb, 10
        )
        self.knife_pose_sub = self.create_subscription(
            Pose, '/toast/knifePose', self.knife_pose_sub_cb, 10
        )
        self.goHome = self.create_service(
            Empty, '/gohome', self.go_home, callback_group=client_cb_group
        )
        self.loaf_tray_pose = None
        self.lever_pose = None
        self.plate_pose = None
        self.knife_pose = None

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
                                         [tableSectionOneDims,
                                             tableSectionTwoDims, pedastleDims],
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
        elif self.loaf_tray_pose is not None:
            # Open the gripper
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Move the gripper to be above a slice of toast
            ########## Set theses value to match real world
            slice1OffsetY = 0.042
            sliceOffsetY = 0.0
            sliceOffsetZ = 0.0
            object_approach_z_offset = 0.1
            ##########

            currentPose = await self.mpi.getCurrentPose()

            self.get_logger().info(f'Attempting pose: {self.loaf_tray_pose.position}')

            goal = [
                self.loaf_tray_pose.position.x,
                self.loaf_tray_pose.position.y + sliceOffsetY * self.breadNumber,
                self.loaf_tray_pose.position.z + object_approach_z_offset + sliceOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Close the gripper
            self.get_logger().debug('Closing Gripper')
            await self.mpi.operateGripper(openGripper=False)

            # # Move the bread out of the loaf holder
            # currentPose = await self.mpi.getCurrentPose()
            # ########## Set theses value to match real world
            # loafHolderOffsetZ = 0.0
            # ##########
            # goal = [
            #     currentPose.pose.position.x,
            #     currentPose.pose.position.y,
            #     currentPose.pose.position.z + loafHolderOffsetZ,
            #     currentPose.pose.orientation.x,
            #     currentPose.pose.orientation.y,
            #     currentPose.pose.orientation.z,
            #     currentPose.pose.orientation.w
            # ]
            # pathType = 'POSE'
            # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            # await self.mpi.planPath(pathType, goal, execute=True)

            # # Move the bread to be directly over the toaster slot
            # ########## Set theses value to match real world
            # ### Offset from lever to toast slot
            # slotOffsetX = 0.0
            # toasterOffsetX = 0.0
            # toasterOffsetY = 0.0
            # toasterOffsetZ = 0.0
            # ##########
            # goal = [
            #     self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
            #     self.lever_pose.position.y + toasterOffsetY,
            #     self.lever_pose.position.z + toasterOffsetZ,
            #     self.lever_pose.orientation.x,
            #     self.lever_pose.orientation.y,
            #     self.lever_pose.orientation.z,
            #     self.lever_pose.orientation.w
            # ]
            # pathType = 'POSE'
            # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            # await self.mpi.planPath(pathType, goal, execute=True)

            # # Drop the bread into the toaster slot
            # # Close the gripper
            # self.get_logger().debug('Opening Gripper')
            # await self.mpi.operateGripper(openGripper=True)

            # # Increment bread number so franka knows which slice to grab
            # self.breadNumber += 1
        return response

    def loaf_tray_pose_sub_cb(self, msg: Pose):
        """
        Update pose of loaf tray.

        :param msg: Loaf tray pose
        :type msg: Pose
        """
        self.loaf_tray_pose = msg

    def lever_pose_sub_cb(self, msg: Pose):
        """
        Update pose of lever.

        :param msg: Lever pose
        :type msg: Pose
        """
        self.lever_pose = msg

    def plate_pose_sub_cb(self, msg: Pose):
        """
        Update pose of plate.

        :param msg: plate pose
        :type msg: Pose
        """
        self.plate_pose = msg

    def knife_pose_sub_cb(self, msg: Pose):
        """
        Update pose of knife.

        :param msg: Knife pose
        :type msg: Pose
        """
        self.knife_pose = msg

    async def go_home(self, request, response):
        """Send robot home."""
        # Move the arm directly above the object
        goal = self.home_joints
        pathType = 'JOINT'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)
        return response


def main(args=None):
    """Run node."""
    rclpy.init(args=args)
    toast_bot = ToastBot()
    rclpy.spin(toast_bot)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
