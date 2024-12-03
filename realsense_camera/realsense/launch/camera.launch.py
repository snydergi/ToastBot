import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='realsense_camera').find('realsense')
    tags_yaml = os.path.join(pkg_share, 'config/tags.yaml')
    pcl_rviz = os.path.join(pkg_share, 'config/pcl.rviz')
    tags_tf_rviz = os.path.join(pkg_share, 'config/tags_tf.rviz')

    # Combine all actions into a LaunchDescription
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            "depth_module.profile",
            default_value="1280x720x30",
            description="Depth module resolution and framerate."
        ),
        DeclareLaunchArgument(
            "rgb_camera.profile",
            default_value="1280x720x30",
            description="RGB camera resolution and framerate."
        ),
        DeclareLaunchArgument(
            "align_depth",
            default_value="true",
            description="Align depth to RGB camera frame."
        ),
        DeclareLaunchArgument(
            "pointcloud.enable",
            default_value="true",
            description="Enable point cloud generation."
        ),

        # Include the RealSense2 Camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare('realsense2_camera'),
                        "launch",
                        "rs_launch.py"
                    ]
                )
            ),
            launch_arguments={
                "depth_module.profile": LaunchConfiguration("depth_module.profile"),
                "rgb_camera.profile": LaunchConfiguration("rgb_camera.profile"),
                "align_depth": LaunchConfiguration("align_depth"),
                "pointcloud.enable": LaunchConfiguration("pointcloud.enable"),
            }.items()
        ),

        # Uncomment and configure these nodes as needed
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                '-d', tags_tf_rviz
            ],
            output="screen",
        ),
        Node(
            package="realsense",
            executable="table",
            output="screen",
            remappings=[
                ("pcl_handler", "/camera/camera/depth/color/points"),
            ],
        ),
        # Node(
        #     package="realsense",
        #     executable="simpleDetection",
        #     output="screen",
        # ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/image_raw')
            ],
            arguments=[
                '--params-file', tags_yaml
            ]
        ),
        # AprilTags detection node
        # Node(
        #     package="realsense",
        #     executable="apriltagsDetection",
        #     output="screen",
        # )
    ])
