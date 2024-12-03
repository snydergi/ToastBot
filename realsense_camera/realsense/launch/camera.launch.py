import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directories
    realsense_share = get_package_share_directory("realsense")
    realsense2_camera_share = get_package_share_directory("realsense2_camera")

    # Combine all actions into a LaunchDescription
    return LaunchDescription([
            DeclareLaunchArgument(
                "depth_module.profile", default_value="1280x720x30",
                description="Depth module resolution and framerate."
            ),
            DeclareLaunchArgument(
                "rgb_camera.profile", default_value="1280x720x30",
                description="RGB camera resolution and framerate."
            ),
            DeclareLaunchArgument(
                "align_depth", default_value="true",
                description="Align depth to RGB camera frame."
            ),
            DeclareLaunchArgument(
                "pointcloud.enable", default_value="true",
                description="Enable point cloud generation."
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense2_camera_share, "launch", "rs_launch.py")
            ),
            launch_arguments={
                    "depth_module.profile": LaunchConfiguration("depth_module.profile"),
                    "rgb_camera.profile": LaunchConfiguration("rgb_camera.profile"),
                    "align_depth": LaunchConfiguration("align_depth"),
                    "pointcloud.enable": LaunchConfiguration("pointcloud.enable"),
                }.items(),
        ),
        Node(
            package="realsense",
            executable="table",
            output="screen",
            remappings=[
                ("pcl_handler", "/camera/camera/depth/color/points"),
            ],
        ),
        Node(
            package="realsense",
            executable="yolo",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(realsense_share, "config", "pcl.rviz")],
            output="screen",
        )
    ])
