"""
Contains services to toast bread in a toaster with the Franka Emika Panda arm.

SUBSCRIBERS:
  + '/toast/loafTrayPose' (geometry_msgs/msg/Pose) - Pose of the loaf tray April Tag
  + '/toast/leverPose' (geometry_msgs/msg/Pose) - Pose of the toaster lever April Tag
  + '/toast/slidePose' (geometry_msgs/msg/Pose) - Pose of the slide April Tag
  + '/toast/brushPose' (geometry_msgs/msg/Pose) - Pose of the brush April Tag
  + '/toast/bowlPose' (geometry_msgs/msg/Pose) - Pose of the butter bowl April Tag

SERVICES:
  + '/buildScene' (std_srvs/srv/Empty) - Create scene collision objects
  + '/breadToToaster' (std_srvs/srv/Empty) - Move bread from loaf into toaster
  + '/actuateLever' (std_srvs/srv/Empty) - Actuate toaster lever
  + '/gohome' (std_srvs/srv/Empty) - Moves to home position
  + '/toastToPlate' (std_srvs/srv/Empty) - Moves toast from toaster to plate
  + '/openGripper' (std_srvs/srv/Empty) - Opens robot gripper
  + '/closeGripper' (std_srvs/srv/Empty) - Closes robot gripper
  + '/initiateToasting' (std_srvs/srv/Empty) - Begin toasting sequence from loaf to actuating lever

"""

from moveitapi.mpi import MotionPlanningInterface
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped


def quaternion_from_euler(ai, aj, ak):
    """
    Convert xyz or ijk angles in radians to their quaternion equivalents.

    :param ai: Angle about the i or x axis in radians.
    :type ai: float
    :param aj: Angle about the j or y axis in radians.
    :type aj: float
    :param ak: Angle about the k or z axis in radians.
    :type ak: float
    :return: An array containing the x,y,z,w quaternion values calculated
    :return: numpy.array()
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class ToastBot(Node):
    """
    Toast bread in a toaster with the Franka Emika Panda arm.

    :param Node: Node object from rclpy.node
    :type Node: rclpy.node.Node()
    """

    def __init__(self):
        """Initialize variables, subscribers, and services."""
        super().__init__('toast_bot')
        self.get_logger().debug('ToastBot Started!')
        # Links 1-7 Joint positions for home position [deg]
        self.home_joints = [0, -45, 0, -135, 0, 90, 45]
        # convert to radians
        self.home_joints = [math.radians(i) for i in self.home_joints]

        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.mpi = MotionPlanningInterface(self)

        # Subscribers
        self.loaf_tray_pose_sub = self.create_subscription(
            Pose, '/toast/loafTrayPose', self.loaf_tray_pose_sub_cb, 10
        )
        self.lever_pose_sub = self.create_subscription(
            Pose, '/toast/leverPose', self.lever_pose_sub_cb, 10
        )
        self.slide_pose_sub = self.create_subscription(
            Pose, '/toast/slidePose', self.slide_pose_sub_cb, 10
        )
        self.brush_pose_sub = self.create_subscription(
            Pose, '/toast/brushPose', self.brush_pose_sub_cb, 10
        )
        self.bowl_pose_sub = self.create_subscription(
            Pose, '/toast/bowlPose', self.bowl_pose_sub_cb, 10
        )
        self.slide_pose_sub = self.create_subscription(
            Pose, '/toast/slidePose', self.slide_pose_sub_cb, 10
        )

        # Services
        self.setScene = self.create_service(
            Empty, '/buildScene', self.setScene_callback,
            callback_group=client_cb_group
        )
        self.breadToToaster = self.create_service(
            Empty, '/breadToToaster', self.breadToToaster_callback, callback_group=client_cb_group
        )
        self.actuateLever = self.create_service(
            Empty, '/actuateLever', self.actuateLever_callback, callback_group=client_cb_group
        )
        self.breadNumber = 1  # So the franka picks the correct piece of bread
        self.goHome = self.create_service(
            Empty, '/gohome', self.go_home, callback_group=client_cb_group
        )
        self.toastToPlate = self.create_service(
            Empty, '/toastToPlate', self.toastToPlate_callback, callback_group=client_cb_group
        )
        self.openGripper = self.create_service(
            Empty, '/openGripper', self.openGripper_callback, callback_group=client_cb_group
        )
        self.closeGripper = self.create_service(
            Empty, '/closeGripper', self.closeGripper_callback, callback_group=client_cb_group
        )
        self.initiateToasting = self.create_service(
            Empty, '/initiateToasting', self.initiateToasting_cb, callback_group=client_cb_group
        )

        # Poses of all the objects in our scenes
        self.loaf_tray_pose = None
        self.lever_pose = None
        self.plate_pose = None
        self.brush_pose = None
        self.bowl_pose = None
        self.slide_pose = None

        # Logic Variables
        self.cartesianAngle = quaternion_from_euler(-np.pi, 0, -np.pi / 4)
        self.leverPrevUp = True
        self.leverCurUp = True
        self.postToastRan = False

        # Create the timer
        self.timer = self.create_timer(1.0, self.timer_cb)

    async def timer_cb(self):
        """Run timer."""
        if self.loaf_tray_pose is not None:
            if self.leverPrevUp is False and \
               self.leverCurUp is True and \
               self.postToastRan is False:
                self.get_logger().info('Post Toasting Time!')
                # self.loop.create_task(self.postToast_cb())
                self.executor.create_task(self.postToast_cb)
                # future = self.executor.create_task(self.postToastButter_cb)

    async def postToast_cb(self):
        """Pull toast from toaster and place on slide."""
        self.postToastRan = True

        self.get_logger().info('Post Toast Called!')
        currentPose: PoseStamped = await self.mpi.getCurrentPose()

        self.get_logger().info('Got current pose!')

        # Open the gripper for the toast
        self.get_logger().debug('Opening Gripper')
        await self.mpi.operateGripper(openGripper=True)

        self.get_logger().info('Opened gripper!')
        toasterOffsetX = 0.0
        toasterOffsetY = 0.005
        toasterOffsetZ = 0.24
        
        goal = [
            # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
            self.lever_pose.position.x + toasterOffsetX,
            self.lever_pose.position.y + toasterOffsetY,
            self.lever_pose.position.z + toasterOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)


        toasterOffsetX = 0.175
        toasterOffsetY = 0.01
        toasterOffsetZ = 0.15
        goal = [
            self.lever_pose.position.x + toasterOffsetX,
            self.lever_pose.position.y + toasterOffsetY,
            self.lever_pose.position.z + toasterOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        self.get_logger().info('Move 1 Called!')

        # Close the gripper on the toast
        self.get_logger().debug('Closing Gripper')
        await self.mpi.operateGripper(openGripper=False)

        # Raise toast out of slot
        toasterOffsetX = 0.175
        toasterOffsetY = 0.01
        toasterOffsetZ = 0.22
        goal = [
            self.lever_pose.position.x + toasterOffsetX,
            self.lever_pose.position.y + toasterOffsetY,
            self.lever_pose.position.z + toasterOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Move the toast to be directly over the plate
        slideOffsetX = -0.11
        slideOffsetY = -0.05
        slideOffsetZ = 0.5
        goal = [
            self.slide_pose.position.x + slideOffsetX,
            self.slide_pose.position.y + slideOffsetY,
            self.slide_pose.position.z + slideOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Move the toast to be closer to the plate
        slideOffsetX = -0.11
        slideOffsetY = -0.05
        slideOffsetZ = 0.175
        goal = [
            self.slide_pose.position.x + slideOffsetX,
            self.slide_pose.position.y + slideOffsetY,
            self.slide_pose.position.z + slideOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # # Open the gripper to drop the toast
        self.get_logger().debug('Opening Gripper')
        await self.mpi.operateGripper(openGripper=True)

        # Move to be high over the plate
        slideOffsetX = -0.11
        slideOffsetY = -0.05
        slideOffsetZ = 0.5
        goal = [
            self.slide_pose.position.x + slideOffsetX,
            self.slide_pose.position.y + slideOffsetY,
            self.slide_pose.position.z + slideOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Return to home position
        goal = self.home_joints
        pathType = 'JOINT'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)
        self.get_logger().info('All done!')
        self.executor.shutdown()
        return []

    async def postToastButter_cb(self):
        """Remove toast from toaster, place on slide, and butter."""
        self.get_logger().info('Post Toast Called!')
        # currentPose = None

        # currentPose: PoseStamped = await self.mpi.getCurrentPose()

        # Post Tost goes here

        # Open the gripper before moving
        self.get_logger().info('Opening the gripper!')
        await self.mpi.operateGripper(openGripper=True)

        self.get_logger().info('Opened gripper!')

        # Move from prep position to the brush
        brushOffsetX = -0.05
        brushOffsetY = -0.03
        brushPrepOffsetZ = 0.24

        quat = quaternion_from_euler(0.0, np.pi, np.pi / 2)

        goal = [
            self.brush_pose.position.x + brushOffsetX,
            self.brush_pose.position.y + brushOffsetY,
            self.brush_pose.position.z + brushPrepOffsetZ,
            quat[0],
            quat[1],
            quat[2],
            quat[3],
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.2)

        # Close the gripper around the brush handle
        self.get_logger().info('Closing the gripper!')
        await self.mpi.operateGripper(openGripper=False)

        self.executor.shutdown()
        return []

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
        """
        Move a piece of bread from the loaf holder to the toaster.

        This function moves the gripper to a piece of bread, grips the piece of bread,
        moves the piece of bread to the toaster, then releasses the bread.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        self.get_logger().info('BreadToToaster Callback called!')
        if self.breadNumber > 4:
            self.get_logger().warn('All out of bread!')
        elif self.loaf_tray_pose is not None:
            # Open the gripper
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Move the gripper to be above a slice of toast
            sliceOffsetY = 0.0
            sliceOffsetZ = 0.0
            object_approach_z_offset = 0.075

            currentPose = await self.mpi.getCurrentPose()

            self.get_logger().info(
                f'Attempting pose: {self.loaf_tray_pose.position}'
            )

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
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.25)

            # Close the gripper
            self.get_logger().debug('Closing Gripper')
            await self.mpi.operateGripper(openGripper=False)

            # Lift bread out of slot
            goal = [
                self.loaf_tray_pose.position.x,
                self.loaf_tray_pose.position.y + sliceOffsetY * self.breadNumber,
                self.loaf_tray_pose.position.z + 4 * object_approach_z_offset + sliceOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.05)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the bread to be directly over the toaster slot
            toasterOffsetX = 0.18
            toasterOffsetY = 0.0005
            toasterOffsetZ = 0.25
            goal = [
                self.lever_pose.position.x + toasterOffsetX,
                self.lever_pose.position.y + toasterOffsetY,
                self.lever_pose.position.z + toasterOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().debug(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the bread into the slot
            toasterOffsetX = 0.19
            toasterOffsetY = 0.0005
            toasterOffsetZ = 0.2
            goal = [
                self.lever_pose.position.x + toasterOffsetX,
                self.lever_pose.position.y + toasterOffsetY,
                self.lever_pose.position.z + toasterOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().debug(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Drop the bread into the toaster slot
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Increment bread number so franka knows which slice to grab
            self.breadNumber += 1
        return response

    async def actuateLever_callback(self, request, response):
        """
        Move a piece of bread from the loaf holder to the toaster.

        This function moves the gripper to the toaster lever and actuates it.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        self.get_logger().info('ActuateLever Callback called!')
        if self.lever_pose is not None:
            # Close the gripper
            self.get_logger().debug('Closing Gripper')
            await self.mpi.operateGripper(openGripper=True)
            await self.mpi.operateGripper(openGripper=False)

            # Move the gripper to be above lever
            leverPrepOffsetX = 0.0
            leverPrepOffsetY = 0.005
            leverPrepOffsetZ = 0.201

            currentPose = await self.mpi.getCurrentPose()

            goal = [
                self.lever_pose.position.x + leverPrepOffsetX,
                self.lever_pose.position.y + leverPrepOffsetY,
                self.lever_pose.position.z + leverPrepOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move to press lever
            leverPressOffsetX = 0.0
            leverPressOffsetY = 0.005
            leverPressOffsetZ = 0.06

            goal = [
                self.lever_pose.position.x + leverPressOffsetX,
                self.lever_pose.position.y + leverPressOffsetY,
                self.lever_pose.position.z + leverPressOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the gripper to be above lever
            leverPrepOffsetX = 0.0
            leverPrepOffsetY = 0.005
            leverPrepOffsetZ = 0.201

            currentPose = await self.mpi.getCurrentPose()

            goal = [
                self.lever_pose.position.x + leverPrepOffsetX,
                self.lever_pose.position.y + leverPrepOffsetY,
                self.lever_pose.position.z + leverPrepOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # open the gripper
            await self.mpi.operateGripper(openGripper=True)

        return response

    async def initiateToasting_cb(self, request, response):
        """
        Move a piece of bread from the loaf holder to the toaster, then actuate lever.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        self.get_logger().info('InitiateToasting Callback called!')
        if self.breadNumber > 4:
            self.get_logger().warn('All out of bread!')
        elif self.loaf_tray_pose is not None:
            # Open the gripper
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Move the gripper to be above a slice of toast
            sliceOffsetY = 0.025
            sliceOffsetZ = 0.0
            object_approach_z_offset = 0.075

            currentPose = await self.mpi.getCurrentPose()

            self.get_logger().info(
                f'Attempting pose: {self.loaf_tray_pose.position}'
            )

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
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Close the gripper
            self.get_logger().debug('Closing Gripper')
            await self.mpi.operateGripper(openGripper=False)

            # Lift bread out of slot
            goal = [
                self.loaf_tray_pose.position.x,
                self.loaf_tray_pose.position.y + sliceOffsetY * self.breadNumber,
                self.loaf_tray_pose.position.z + 4 * object_approach_z_offset + sliceOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.05)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the bread to be directly over the toaster slot
            toasterOffsetX = 0.18
            toasterOffsetY = 0.0005
            toasterOffsetZ = 0.26
            goal = [
                self.lever_pose.position.x + toasterOffsetX,
                self.lever_pose.position.y + toasterOffsetY,
                self.lever_pose.position.z + toasterOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the bread into the slot
            toasterOffsetX = 0.175
            toasterOffsetY = 0.01
            toasterOffsetZ = 0.20
            goal = [
                self.lever_pose.position.x + toasterOffsetX,
                self.lever_pose.position.y + toasterOffsetY,
                self.lever_pose.position.z + toasterOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
            ]
            pathType = 'POSE'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Drop the bread into the toaster slot
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().info(
                f'MPI PlanPath pT:{pathType} \n goal:{goal}'
            )
            await self.mpi.planPath(pathType, goal, execute=True)

            # Increment bread number so franka knows which slice to grab
            self.breadNumber += 1

            if self.lever_pose is not None:
                # Close the gripper
                self.get_logger().debug('Closing Gripper')
                await self.mpi.operateGripper(openGripper=False)

                # Move the gripper to be above lever
                leverPrepOffsetX = 0.0
                leverPrepOffsetY = 0.005
                leverPrepOffsetZ = 0.201

                currentPose = await self.mpi.getCurrentPose()

                goal = [
                    self.lever_pose.position.x + leverPrepOffsetX,
                    self.lever_pose.position.y + leverPrepOffsetY,
                    self.lever_pose.position.z + leverPrepOffsetZ,
                    currentPose.pose.orientation.x,
                    currentPose.pose.orientation.y,
                    currentPose.pose.orientation.z,
                    currentPose.pose.orientation.w
                ]
                pathType = 'POSE'
                self.get_logger().info(
                    f'MPI PlanPath pT:{pathType} \n goal:{goal}'
                )
                await self.mpi.planPath(pathType, goal, execute=True)

                # Move to press lever
                leverPressOffsetX = 0.0
                leverPressOffsetY = 0.005
                leverPressOffsetZ = 0.0575

                goal = [
                    self.lever_pose.position.x + leverPressOffsetX,
                    self.lever_pose.position.y + leverPressOffsetY,
                    self.lever_pose.position.z + leverPressOffsetZ,
                    currentPose.pose.orientation.x,
                    currentPose.pose.orientation.y,
                    currentPose.pose.orientation.z,
                    currentPose.pose.orientation.w
                ]
                pathType = 'POSE'
                self.get_logger().info(
                    f'MPI PlanPath pT:{pathType} \n goal:{goal}'
                )
                await self.mpi.planPath(pathType, goal, execute=True)

                # Move the gripper to be above lever
                leverPrepOffsetX = 0.0
                leverPrepOffsetY = 0.005
                leverPrepOffsetZ = 0.201
                currentPose = await self.mpi.getCurrentPose()
                goal = [
                    self.lever_pose.position.x + leverPrepOffsetX,
                    self.lever_pose.position.y + leverPrepOffsetY,
                    self.lever_pose.position.z + leverPrepOffsetZ,
                    currentPose.pose.orientation.x,
                    currentPose.pose.orientation.y,
                    currentPose.pose.orientation.z,
                    currentPose.pose.orientation.w
                ]
                pathType = 'POSE'
                self.get_logger().info(
                    f'MPI PlanPath pT:{pathType} \n goal:{goal}'
                )
                await self.mpi.planPath(pathType, goal, execute=True)

                # Return to home position
                goal = self.home_joints
                pathType = 'JOINT'
                self.get_logger().info(
                    f'MPI PlanPath pT:{pathType} \n goal:{goal}'
                )
                await self.mpi.planPath(pathType, goal, execute=True)

        return response

    async def toastToPlate_callback(self, request, response):
        """
        Move a piece of bread from toaster to the plate.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        currentPose: PoseStamped = await self.mpi.getCurrentPose()

        # Open the gripper for the toast
        self.get_logger().debug('Opening Gripper')
        await self.mpi.operateGripper(openGripper=True)

        # Move the bread to be directly over the toaster slot
        toasterOffsetX = 0.0
        toasterOffsetY = 0.005
        toasterOffsetZ = 0.24
        goal = [
            self.lever_pose.position.x + toasterOffsetX,
            self.lever_pose.position.y + toasterOffsetY,
            self.lever_pose.position.z + toasterOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Move to grab toast from slot
        toasterOffsetX = 0.19
        toasterOffsetY = 0.0005
        toasterOffsetZ = 0.165
        goal = [
            self.lever_pose.position.x + toasterOffsetX,
            self.lever_pose.position.y + toasterOffsetY,
            self.lever_pose.position.z + toasterOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Close the gripper on the toast
        self.get_logger().debug('Closing Gripper')
        await self.mpi.operateGripper(openGripper=False)

        # Move the toast to be directly over the toaster slot
        toasterOffsetX = 0.19
        toasterOffsetY = 0.0005
        toasterOffsetZ = 0.28
        goal = [
            self.lever_pose.position.x + toasterOffsetX,
            self.lever_pose.position.y + toasterOffsetY,
            self.lever_pose.position.z + toasterOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Move the toast to be directly over the plate
        plateOffsetX = 0.0
        plateOffsetY = 0.0
        plateOffsetZ = 0.5
        goal = [
            self.slide_pose.position.x + plateOffsetX,
            self.slide_pose.position.y + plateOffsetY,
            self.slide_pose.position.z + plateOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Move the toast to be closer to the plate
        plateOffsetX = 0.0
        plateOffsetY = 0.025
        plateOffsetZ = 0.175
        goal = [
            self.slide_pose.position.x + plateOffsetX,
            self.slide_pose.position.y + plateOffsetY,
            self.slide_pose.position.z + plateOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Open the gripper to drop the toast
        self.get_logger().debug('Opening Gripper')
        await self.mpi.operateGripper(openGripper=True)

        # Guide the toast release to the plate
        plateOffsetX = 0.0
        plateOffsetY = 0.1
        plateOffsetZ = 0.175
        goal = [
            self.slide_pose.position.x + plateOffsetX,
            self.slide_pose.position.y + plateOffsetY,
            self.slide_pose.position.z + plateOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Return to home position
        goal = self.home_joints
        pathType = 'JOINT'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)
        return response

    async def openGripper_callback(self, request, response):
        """
        Open gripper.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        self.get_logger().debug('Opening Gripper')
        await self.mpi.operateGripper(openGripper=True)
        return response

    async def closeGripper_callback(self, request, response):
        """
        Close gripper.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        self.get_logger().debug('Closing Gripper')
        await self.mpi.operateGripper(openGripper=False)
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
        # self.get_logger().info('Updated Lever Pose!')
        self.lever_pose = msg
        if self.leverPrevUp and self.lever_pose.position.z < 0.25:
            self.leverPrevUp = False
            self.leverCurUp = False
        elif not self.leverPrevUp and self.lever_pose.position.z > 0.25:
            self.leverCurUp = True

    def slide_pose_sub_cb(self, msg: Pose):
        """
        Update pose of plate.

        :param msg: plate pose
        :type msg: Pose
        """
        self.slide_pose = msg

    def brush_pose_sub_cb(self, msg: Pose):
        """
        Update pose of brush holder.

        :param msg: Brush pose
        :type msg: Pose
        """
        self.brush_pose = msg

    def bowl_pose_sub_cb(self, msg: Pose):
        """
        Update pose of bowl.

        :param msg: Bowl pose
        :type msg: Pose
        """
        self.bowl_pose = msg

    async def go_home(self, request, response):
        """
        Send robot to home position.

        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
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
