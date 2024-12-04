"""
AprilTags Detection and Visualization Node

This script defines a ROS 2 node for detecting AprilTags in a live camera feed, drawing 
visual markers on the detected tags, and providing relevant logging information. It uses 
the `sensor_msgs/Image` topic for camera data and the `apriltag_msgs/AprilTagDetectionArray` 
topic for AprilTag detection messages.

Classes:
--------
- `FrameListener`: A custom ROS 2 node that listens for transformations between frames 
  in a turtlesim simulation. It spawns a new turtle (`turtle2`) and sends velocity 
  commands for it to follow another frame (`turtle1` by default).

Methods:
--------
- `__init__()`: Initializes the node, sets up transformation listeners, and declares 
  parameters. Also initializes a client for spawning turtles and a publisher for turtle 
  velocity commands.
- `on_timer()`: Periodically checks and computes the transformation between the target 
  frame and `turtle2`, commanding `turtle2` to follow the target frame. Handles spawning 
  of `turtle2` if it has not yet been spawned.

Main Execution:
---------------
- Initializes the ROS 2 node.
- Declares a `target_frame` parameter (default is `turtle1`).
- Uses the tf2 ROS library to compute transformations and control the movement of `turtle2`.
- Spawns `turtle2` in the simulation environment using the `Spawn` service.
- Spins the node to keep it running and responsive to updates.

Dependencies:
-------------
- ROS 2
- tf2_ros (for transformation calculations)
- turtlesim (for simulation environment and services)
- geometry_msgs (for sending velocity commands)

Usage:
------
Run this node in a ROS 2 environment where turtlesim is running. This node spawns a 
second turtle (`turtle2`) and makes it follow the frame of another turtle (`turtle1` 
by default).

Topics:
-------
- Subscribed:
  - `/tf`: Transformation data for frames.
- Published:
  - `turtle2/cmd_vel`: Velocity commands for `turtle2`.
"""



import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'turtle1').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
                try:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)

                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)

                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()