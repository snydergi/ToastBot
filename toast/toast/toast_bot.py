"""TODO."""

from moveitapi.mpi import MotionPlanningInterface
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.executors import MultiThreadedExecutor
import concurrent.futures


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
    """TODO."""

    def __init__(self, executor):
        """TODO."""
        super().__init__('toast_bot')
        self.get_logger().info('ToastBot Started!')
        # Links 1-7 Joint positions for home position [deg]
        self.home_joints = [0, -45, 0, -135, 0, 90, 45]
        # convert to radians
        self.home_joints = [math.radians(i) for i in self.home_joints]

        # self.loop = asyncio.get_event_loop()

        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.mpi = MotionPlanningInterface(self)
        self.setScene = self.create_service(Empty, '/buildScene', self.setScene_callback,
                                            callback_group=client_cb_group)
        self.breadToToaster = self.create_service(
            Empty, '/breadToToaster', self.breadToToaster_callback, callback_group=client_cb_group
        )
        self.actuateLever = self.create_service(
            Empty, '/actuateLever', self.actuateLever_callback, callback_group=client_cb_group
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
        self.brush_pose_sub = self.create_subscription(
            Pose, '/toast/brushPose', self.brush_pose_sub_cb, 10
        )
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
        # self.postToast = self.create_service(
        #     Empty, '/postToast', self.postToast_cb, callback_group=client_cb_group
        # )
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.loaf_tray_pose = None
        self.lever_pose = None
        self.plate_pose = None
        self.brush_pose = None
        self.cartesianAngle = quaternion_from_euler(-np.pi, 0, -np.pi / 4)
        self.leverPrevUp = True
        self.leverCurUp = True
        self.postToastRan = False

        
        self.executor = executor
        self.executor.add_node(self)

    async def timer_cb(self):
        """Run timer."""
        if self.loaf_tray_pose is not None:
            if self.leverPrevUp is False and \
               self.leverCurUp is True and \
               self.postToastRan is False:
                self.get_logger().info('Post Toasting Time!')
                # self.loop.create_task(self.postToast_cb())
                await self.postToast_cb()

    async def postToast_cb(self):
        """TODO."""
        self.postToastRan = True
        
        

        self.get_logger().info('Post Toast Called!')
        currentPose = None

        if currentPose is None:
            self.executor.spin_once(timeout_sec=0.1)
            currentPose: PoseStamped = self.mpi.getCurrentPose()
            self.get_logger().info('Calling!')
            
        self.get_logger().info('awaited current pose!')
        # rclpy.spin_until_future_complete(currentPose_future)

        gripperState = None

        self.get_logger().info('Opening the gripper!')
        if gripperState is None:
            self.executor.spin_once(timeout_sec=0.1)
            await self.mpi.operateGripper(openGripper=True)
            self.get_logger().info('Calling the gripper')

        self.get_logger().info('Opened gripper!')

        # Move the bread to be directly over the toaster slot
        ########## Set theses value to match real world
        ### Offset from lever to toast slot
        # slotOffsetX = 0.0
        toasterOffsetX = 0.0
        toasterOffsetY = 0.005
        toasterOffsetZ = 0.24
        ##########
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
        move_over_plate =  None

        while move_over_plate is None:
            self.executor.spin_once(timeout_sec=0.1)
            move_over_plate = self.mpi.planPath(pathType, goal, execute=True)
        self.get_logger().info('awaited move over plate!')

        self.get_logger().info('Move 1 Called!')

        # Move to grab toast from slot
        ######### Set theses value to match real world
        ## Offset from lever to toast slot
        # slotOffsetX = 0.0
        # toasterOffsetX = 0.19
        # toasterOffsetY = 0.0005
        # toasterOffsetZ = 0.165
        # ##########
        # goal = [
        #     # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
        #     self.lever_pose.position.x + toasterOffsetX,
        #     self.lever_pose.position.y + toasterOffsetY,
        #     self.lever_pose.position.z + toasterOffsetZ,
        #     currentPose.pose.orientation.x,
        #     currentPose.pose.orientation.y,
        #     currentPose.pose.orientation.z,
        #     currentPose.pose.orientation.w
        # ]
        # pathType = 'POSE'
        # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        # await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.05)

        # # Close the gripper on the toast
        # self.get_logger().debug('Closing Gripper')
        # await self.mpi.operateGripper(openGripper=False)

        # # Move the toast to be directly over the toaster slot
        # ########## Set theses value to match real world
        # ### Offset from lever to toast slot
        # # slotOffsetX = 0.0
        # toasterOffsetX = 0.19
        # toasterOffsetY = 0.0005
        # toasterOffsetZ = 0.28
        # ##########
        # goal = [
        #     # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
        #     self.lever_pose.position.x + toasterOffsetX,
        #     self.lever_pose.position.y + toasterOffsetY,
        #     self.lever_pose.position.z + toasterOffsetZ,
        #     currentPose.pose.orientation.x,
        #     currentPose.pose.orientation.y,
        #     currentPose.pose.orientation.z,
        #     currentPose.pose.orientation.w
        # ]
        # pathType = 'POSE'
        # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        # await self.mpi.planPath(pathType, goal, execute=True)

        # # Move the toast to be directly over the plate
        # ########## Set theses value to match real world
        # ### Offset from lever to toast slot
        # # slotOffsetX = 0.0
        # plateOffsetX = 0.0
        # plateOffsetY = 0.0
        # plateOffsetZ = 0.5
        # ##########
        # goal = [
        #     # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
        #     self.plate_pose.position.x + plateOffsetX,
        #     self.plate_pose.position.y + plateOffsetY,
        #     self.plate_pose.position.z + plateOffsetZ,
        #     currentPose.pose.orientation.x,
        #     currentPose.pose.orientation.y,
        #     currentPose.pose.orientation.z,
        #     currentPose.pose.orientation.w
        # ]
        # pathType = 'POSE'
        # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        # await self.mpi.planPath(pathType, goal, execute=True)

        # # Move the toast to be closer to the plate
        # ########## Set theses value to match real world
        # ### Offset from lever to toast slot
        # # slotOffsetX = 0.0
        # plateOffsetX = 0.0
        # plateOffsetY = 0.025
        # plateOffsetZ = 0.175
        # ##########
        # goal = [
        #     # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
        #     self.plate_pose.position.x + plateOffsetX,
        #     self.plate_pose.position.y + plateOffsetY,
        #     self.plate_pose.position.z + plateOffsetZ,
        #     currentPose.pose.orientation.x,
        #     currentPose.pose.orientation.y,
        #     currentPose.pose.orientation.z,
        #     currentPose.pose.orientation.w
        # ]
        # pathType = 'POSE'
        # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        # await self.mpi.planPath(pathType, goal, execute=True)

        # # Open the gripper to drop the toast
        # self.get_logger().debug('Opening Gripper')
        # await self.mpi.operateGripper(openGripper=True)

        # # Guide the toast release to the plate
        # ########## Set theses value to match real world
        # ### Offset from lever to toast slot
        # # slotOffsetX = 0.0
        # plateOffsetX = 0.0
        # plateOffsetY = 0.1
        # plateOffsetZ = 0.175
        # ##########
        # goal = [
        #     # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
        #     self.plate_pose.position.x + plateOffsetX,
        #     self.plate_pose.position.y + plateOffsetY,
        #     self.plate_pose.position.z + plateOffsetZ,
        #     currentPose.pose.orientation.x,
        #     currentPose.pose.orientation.y,
        #     currentPose.pose.orientation.z,
        #     currentPose.pose.orientation.w
        # ]
        # pathType = 'POSE'
        # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        # await self.mpi.planPath(pathType, goal, execute=True)

        # # Return to home position
        # goal = self.home_joints
        # pathType = 'JOINT'
        # self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        # await self.mpi.planPath(pathType, goal, execute=True)
        # self.get_logger().info('All done!')
        self.executor.shutdown()
        return []
    
    async def postToastButter_cb(self):
        """Post toast callback with buttering."""
        
        self.get_logger().info('Post Toast Called!')
        currentPose = None

        if currentPose is None:
            self.executor.spin_once(timeout_sec=0.1)
            currentPose: PoseStamped = self.mpi.getCurrentPose()
            self.get_logger().info('Calling!')
            
        ##### Post Tost goes here
        
        # Open the gripper before moving
        self.get_logger().info('Opening the gripper!')
        if gripperState is None:
            self.executor.spin_once(timeout_sec=0.1)
            await self.mpi.operateGripper(openGripper=True)
            self.get_logger().info('Calling the gripper')

        self.get_logger().info('Opened gripper!')
        
        # Move from home position to the brush
        ## Offsets from april tag to brush handle
        brushOffsetX = 0.0
        brushOffsetY = 0.07
        brushOffsetZ = 0.150
        
        goal = [
            self.brush_pose.position.x + brushOffsetX,
            self.brush_pose.position.y + brushOffsetY,
            self.brush_pose.position.z + brushOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        
        # Close the gripper around the brush handle
        self.get_logger().info('Closing the gripper!')
        if gripperState is None:
            self.executor.spin_once(timeout_sec=0.1)
            await self.mpi.operateGripper(openGripper=False)
            self.get_logger().info('Calling the gripper')

        self.get_logger().info('Closed gripper!')
        


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
            #       Set theses value to match real world
            # slice1OffsetY = 0.042
            sliceOffsetY = 0.0
            sliceOffsetZ = 0.0
            object_approach_z_offset = 0.075
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
            self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
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
            self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.05)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the bread to be directly over the toaster slot
            ########## Set theses value to match real world
            ### Offset from lever to toast slot
            # slotOffsetX = 0.0
            toasterOffsetX = 0.18
            toasterOffsetY = 0.0005
            toasterOffsetZ = 0.25
            ##########
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

            # Move the bread into the slot
            ########## Set theses value to match real world
            ### Offset from lever to toast slot
            # slotOffsetX = 0.0
            toasterOffsetX = 0.19
            toasterOffsetY = 0.0005
            toasterOffsetZ = 0.2
            ##########
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

            # Drop the bread into the toaster slot
            # Open the gripper
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Increment bread number so franka knows which slice to grab
            self.breadNumber += 1
        return response

    async def actuateLever_callback(self, request, response):
        """Move a piece of bread from the loaf holder to the toaster.

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
            ########## Set theses value to match real world
            # slice1OffsetY = 0.042
            leverPrepOffsetX = 0.0
            leverPrepOffsetY = 0.005
            leverPrepOffsetZ = 0.201
            ##########

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
            self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move to press lever
            leverPressOffsetX = 0.0
            leverPressOffsetY = 0.005
            leverPressOffsetZ = 0.045

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
            self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the gripper to be above lever
            ########## Set theses value to match real world
            # slice1OffsetY = 0.042
            leverPrepOffsetX = 0.0
            leverPrepOffsetY = 0.005
            leverPrepOffsetZ = 0.201
            ##########

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
            self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # open the gripper
            await self.mpi.operateGripper(openGripper=True)

        return response

    async def initiateToasting_cb(self, request, response):
        """TODO."""
        self.get_logger().info('BreadToToaster Callback called!')
        if self.breadNumber > 4:
            self.get_logger().warn('All out of bread!')
        elif self.loaf_tray_pose is not None:
            # Open the gripper
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Move the gripper to be above a slice of toast
            #       Set theses value to match real world
            # slice1OffsetY = 0.042
            sliceOffsetY = 0.0
            sliceOffsetZ = 0.0
            object_approach_z_offset = 0.075
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
            self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            # await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.25)
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
            self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            #await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.05)
            await self.mpi.planPath(pathType, goal, execute=True)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Move the bread to be directly over the toaster slot
            ########## Set theses value to match real world
            ### Offset from lever to toast slot
            # slotOffsetX = 0.0
            toasterOffsetX = 0.18
            toasterOffsetY = 0.0005
            toasterOffsetZ = 0.25
            ##########
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

            # Move the bread into the slot
            ########## Set theses value to match real world
            ### Offset from lever to toast slot
            # slotOffsetX = 0.0
            toasterOffsetX = 0.19
            toasterOffsetY = 0.0005
            toasterOffsetZ = 0.2
            ##########
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

            # Drop the bread into the toaster slot
            # Open the gripper
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)

            # Return to home position
            goal = self.home_joints
            pathType = 'JOINT'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)

            # Increment bread number so franka knows which slice to grab
            self.breadNumber += 1

            if self.lever_pose is not None:
                # Close the gripper
                self.get_logger().debug('Closing Gripper')
                await self.mpi.operateGripper(openGripper=True)
                await self.mpi.operateGripper(openGripper=False)

                # Move the gripper to be above lever
                ########## Set theses value to match real world
                # slice1OffsetY = 0.042
                leverPrepOffsetX = 0.0
                leverPrepOffsetY = 0.005
                leverPrepOffsetZ = 0.201
                ##########

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
                self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
                await self.mpi.planPath(pathType, goal, execute=True)

                # Move to press lever
                leverPressOffsetX = 0.0
                leverPressOffsetY = 0.005
                leverPressOffsetZ = 0.048

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
                self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
                await self.mpi.planPath(pathType, goal, execute=True)

                # Move the gripper to be above lever
                ########## Set theses value to match real world
                # slice1OffsetY = 0.042
                leverPrepOffsetX = 0.0
                leverPrepOffsetY = 0.005
                leverPrepOffsetZ = 0.201
                ##########

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
                self.get_logger().info(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
                await self.mpi.planPath(pathType, goal, execute=True)

                # Return to home position
                goal = self.home_joints
                pathType = 'JOINT'
                self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
                await self.mpi.planPath(pathType, goal, execute=True)

        return response

    async def toastToPlate_callback(self, request, response):
        """TODO."""
        currentPose: PoseStamped = await self.mpi.getCurrentPose()

        # Open the gripper for the toast
        self.get_logger().debug('Opening Gripper')
        await self.mpi.operateGripper(openGripper=True)

        # Move the bread to be directly over the toaster slot
        ########## Set theses value to match real world
        ### Offset from lever to toast slot
        # slotOffsetX = 0.0
        toasterOffsetX = 0.0
        toasterOffsetY = 0.005
        toasterOffsetZ = 0.24
        ##########
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

        # Move to grab toast from slot
        ######### Set theses value to match real world
        ## Offset from lever to toast slot
        slotOffsetX = 0.0
        toasterOffsetX = 0.19
        toasterOffsetY = 0.0005
        toasterOffsetZ = 0.165
        ##########
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
        # await self.mpi.planPath(pathType, goal, execute=True, velocity_scaling=0.05)
        await self.mpi.planPath(pathType, goal, execute=True)

        # Close the gripper on the toast
        self.get_logger().debug('Closing Gripper')
        await self.mpi.operateGripper(openGripper=False)

        # Move the toast to be directly over the toaster slot
        ########## Set theses value to match real world
        ### Offset from lever to toast slot
        # slotOffsetX = 0.0
        toasterOffsetX = 0.19
        toasterOffsetY = 0.0005
        toasterOffsetZ = 0.28
        ##########
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

        # Move the toast to be directly over the plate
        ########## Set theses value to match real world
        ### Offset from lever to toast slot
        # slotOffsetX = 0.0
        plateOffsetX = 0.0
        plateOffsetY = 0.0
        plateOffsetZ = 0.5
        ##########
        goal = [
            # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
            self.plate_pose.position.x + plateOffsetX,
            self.plate_pose.position.y + plateOffsetY,
            self.plate_pose.position.z + plateOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        ]
        pathType = 'POSE'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)

        # Move the toast to be closer to the plate
        ########## Set theses value to match real world
        ### Offset from lever to toast slot
        # slotOffsetX = 0.0
        plateOffsetX = 0.0
        plateOffsetY = 0.025
        plateOffsetZ = 0.175
        ##########
        goal = [
            # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
            self.plate_pose.position.x + plateOffsetX,
            self.plate_pose.position.y + plateOffsetY,
            self.plate_pose.position.z + plateOffsetZ,
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
        ########## Set theses value to match real world
        ### Offset from lever to toast slot
        # slotOffsetX = 0.0
        plateOffsetX = 0.0
        plateOffsetY = 0.1
        plateOffsetZ = 0.175
        ##########
        goal = [
            # self.lever_pose.position.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
            self.plate_pose.position.x + plateOffsetX,
            self.plate_pose.position.y + plateOffsetY,
            self.plate_pose.position.z + plateOffsetZ,
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
        """TODO."""
        self.get_logger().debug('Opening Gripper')
        await self.mpi.operateGripper(openGripper=True)
        return response

    async def closeGripper_callback(self, request, response):
        """TODO."""
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

    def plate_pose_sub_cb(self, msg: Pose):
        """
        Update pose of plate.

        :param msg: plate pose
        :type msg: Pose
        """
        self.plate_pose = msg

    def brush_pose_sub_cb(self, msg: Pose):
        """
        Update pose of brush.

        :param msg: Brush pose
        :type msg: Pose
        """
        self.brush_pose = msg

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
    # loop = asyncio.get_event_loop()
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    
    toast_bot = ToastBot(executor=executor)
    
    rclpy.spin(toast_bot)
    rclpy.shutdown()
    # pending = asyncio.all_tasks(loop=loop)
    # loop.run_until_complete(asyncio.gather(*pending))
    # loop.close()


if __name__ == '__main__':
    main()
