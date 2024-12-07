"""TODO."""
from math import radians

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
        self.pushToasterLever = self.create_service(Empty, 'pushToasterLever', self.pushToasterLever_callback,
                                                    callback_group=client_cb_group)
        self.breadNumber = 1 # So the franka picks the correct piece of bread
        # Transform Listener
        self.tfBuffer = Buffer()
        self.tfListeneristener = TransformListener(self.tfBuffer, self)
        # Links 1-7 Joint positions for home position [deg]
        self.home_joints = [0, -45, 0, -135, 0, 90, 45]
        # convert to radians
        self.home_joints = [radians(i) for i in self.home_joints]
        

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
        
        breadSliceDimensions = [0.133, 0.110, 20]
        
        slice1 = Pose()

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
                baseLoafTrans = self.tfBuffer.lookup_transform(
                        'base_link', 'loaf_holder', rclpy.time.Time())  ########### Is this the correct transform?
            except tf2_ros.LookupException as e:
                    self.get_logger().info(f'Tranform lookup exception: {e}')
            except tf2_ros.ConnectivityException as e:
                    self.get_logger().info(f'Transform connectivity exception: {e}')
            except tf2_ros.ExtrapolationException as e:
                    self.get_logger().info(f'Transform extrapolation exception: {e}')
            ########## Set theses value to match real world
            sliceOffsetX = 0.0
            sliceOffsetZ = 0.0
            ##########
            goal = [
                baseLoafTrans.transform.translation.x + sliceOffsetX * self.breadNumber,
                baseLoafTrans.transform.translation.y,
                baseLoafTrans.transform.translation.z + sliceOffsetZ,
                baseLoafTrans.transform.rotation.x,
                baseLoafTrans.transform.rotation.y,
                baseLoafTrans.transform.rotation.z,
                baseLoafTrans.transform.rotation.w
                ]
            pathType = 'CARTESIAN'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)
            
            # Close the gripper
            self.get_logger().debug('Closing Gripper')
            await self.mpi.operateGripper(openGripper=False)
            
            # Attach piece of bread to the end effector, remove from scene 
            self.get_logger().debug('Attach bread to robot.')
            await self.mpi.attachObject('Slice_1')
            await self.mpi.removeObjFromScene('Slice_1')

            # Move the bread out of the loaf holder
            currentPose = await self.mpi.getCurrentPose()
            ########## Set theses value to match real world
            loafHolderOffsetZ = 0.0
            ##########
            goal = [
                currentPose.pose.position.x,
                currentPose.pose.position.y,
                currentPose.pose.position.z + loafHolderOffsetZ,
                currentPose.pose.orientation.x,
                currentPose.pose.orientation.y,
                currentPose.pose.orientation.z,
                currentPose.pose.orientation.w
                ]
            pathType = 'CARTESIAN'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)
            
            # Move the bread to be directly over the toaster slot
            try:
                baseToasterTrans = self.tfBuffer.lookup_transform(
                        'base_link', 'toaster', rclpy.time.Time())  ########### Is this the correct transform?
            except tf2_ros.LookupException as e:
                    self.get_logger().info(f'Tranform lookup exception: {e}')
            except tf2_ros.ConnectivityException as e:
                    self.get_logger().info(f'Transform connectivity exception: {e}')
            except tf2_ros.ExtrapolationException as e:
                    self.get_logger().info(f'Transform extrapolation exception: {e}')
            ########## Set theses value to match real world
            slotOffsetX = 0.0
            toasterOffsetX = 0.0
            toasterOffsetY = 0.0
            toasterOffsetZ = 0.0
            ##########
            goal = [
                baseToasterTrans.transform.translation.x + toasterOffsetX + self.breadNumber % 2 * slotOffsetX,
                baseToasterTrans.transform.translation.y + toasterOffsetY,
                baseToasterTrans.transform.translation.z + toasterOffsetZ,
                baseToasterTrans.transform.rotation.x,
                baseToasterTrans.transform.rotation.y,
                baseToasterTrans.transform.rotation.z,
                baseToasterTrans.transform.rotation.w
                ]
            pathType = 'CARTESIAN'
            self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
            await self.mpi.planPath(pathType, goal, execute=True)
            
            # Drop the bread into the toaster slot
            self.get_logger().debug('Opening Gripper')
            await self.mpi.operateGripper(openGripper=True)
            
            # Remove the collision object from the end effector
            await self.mpi.detachObject('Slice_1')
            
            # Increment bread number so franka knows which slice to grab
            self.breadNumber += 1 
        return response
    
    
    async def pushToasterLever_callback(self, request, response):
        """Makes the franka push the toaster lever.
        
        This funtion closes the gripper, moves it to the lever, 
        pushes the lever down, and moves to ready position.
        
        :param request: The request object, typically an empty placeholder for this operation.
        :type request: std_msgs/Empty
        :param response: The response object to be returned after completing the operation.
        :type response: std_msgs/Empty
        """
        self.get_logger().info("PushToasterLever Callback called!")
        
        # Close the gripper
        self.get_logger().debug('Closing Gripper')
        await self.mpi.operateGripper(openGripper=False)
        
        # Move the gripper above the lever
        try:
            baseLeverTrans = self.tfBuffer.lookup_transform(
                    'base_link', 'toaster_lever', rclpy.time.Time())  ########### Is this the correct transform?
        except tf2_ros.LookupException as e:
            self.get_logger().info(f'Tranform lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().info(f'Transform connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().info(f'Transform extrapolation exception: {e}')
        ########## Replace these to match the real world
        toasterLeverOffsetX = 0.0
        toasterLeverOffsetY = 0.0
        toasterLeverOffsetZ = 0.0
        toasterLeverOrientationX = 0.0
        toasterLeverOrientationY = 0.0
        toasterLeverOrientationZ = 0.0
        toasterLeverOrientationW = 0.0
        ##########
        goal = [
        baseLeverTrans.transform.translation.x + toasterLeverOffsetX,
        baseLeverTrans.transform.translation.y + toasterLeverOffsetY,
        baseLeverTrans.transform.translation.z + toasterLeverOffsetZ,
        toasterLeverOrientationX,
        toasterLeverOrientationY,
        toasterLeverOrientationZ,
        toasterLeverOrientationW
        ]
        pathType = 'CARTESIAN'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)
            
        
        
        # Attach the lever collision object to the end effector, remove from scene
        self.get_logger().debug('Attach lever to robot.')
        await self.mpi.attachObject('toasterLever')
        await self.mpi.removeObjFromScene('toasterLever')
        
        # Push the lever down
        currentPose = await self.mpi.getCurrentPose()
        ########## Set theses value to match real world
        loafHolderOffsetZ = 0.0
        ##########
        goal = [
            currentPose.pose.position.x,
            currentPose.pose.position.y,
            currentPose.pose.position.z + loafHolderOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
            ]
        pathType = 'CARTESIAN'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)
        
        # Remove the lever collision object from the end effector
        await self.mpi.detachObject('toasterLever')
        
        # Move the gripper up
        currentPose = await self.mpi.getCurrentPose()
        ########## Set theses value to match real world
        loafHolderOffsetZ = 0.0
        ##########
        goal = [
            currentPose.pose.position.x,
            currentPose.pose.position.y,
            currentPose.pose.position.z + loafHolderOffsetZ,
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
            ]
        pathType = 'CARTESIAN'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)
        
        # Move to ready position
        self._go_home(self)
        
        return response
    
    
    async def _go_home(self):
        """TODO."""
        # Move the arm directly above the object
        goal = self.home_joints
        pathType = 'JOINT'
        self.get_logger().debug(f'MPI PlanPath pT:{pathType} \n goal:{goal}')
        await self.mpi.planPath(pathType, goal, execute=True)



def main(args=None):
    """Run node."""
    rclpy.init(args=args)
    toastBot = ToastBot()
    rclpy.spin(toastBot)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
