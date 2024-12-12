"""
Launch Description Script for RealSense and AprilTag Detection.

This script defines a launch file for initializing a RealSense camera, configuring
its parameters, and setting up related nodes for point cloud processing and AprilTag
detection. It includes the following key components

"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Init function to launch the files."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ]
                )
            ),
            launch_arguments={
                'depth_module.profile': '1280x720x30',
                'rgb_camera.profile': '1280x720x30',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
            }.items()
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info')
            ],
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('realsense'), 'tags.yaml']),
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        Node(
            package='realsense',
            executable='camera_localizer'
        )
    ])
