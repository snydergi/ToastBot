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
    pkg_share = FindPackageShare(package='realsense').find('realsense')
    tags_yaml = os.path.join(pkg_share, 'cfg', 'config/tags.yaml')
    tags_tf_rviz = os.path.join(pkg_share, 'config/tags_tf.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'align_depth',
            default_value='true',
            description='Align depth to RGB camera frame.'
        ),
        DeclareLaunchArgument(
            'pointcloud.enable',
            default_value='true',
            description='Enable point cloud generation.'
        ),

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
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', tags_tf_rviz
            ],
            output='screen',
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
            parameters=[tags_yaml]
        ),
        Node(
            package='realsense',
            executable='apriltagTracker',
            output='screen',
        )
    ])
