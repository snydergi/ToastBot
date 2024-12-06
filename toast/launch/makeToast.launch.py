"""Launch necessary nodes to make toast."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    """TODO."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'demo',
                default_value='True',
                description='Demo Only? (True will launch Franka demo.launch.py)',
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_fer_moveit_config'),
                        'launch',
                        'demo.launch.py',
                    ]
                ),
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('demo'), 'True')
                ),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare('realsense'), 'camera.launch.py',
                    ]
                ),
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('demo'), 'True')
                ),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d', PathJoinSubstitution(
                        [FindPackageShare('toast'), 'config/toast.rviz']
                    )
                ],
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('demo'), 'False')
                ),
            ),
            Node(package='toast', executable='toast_bot'),
        ]
    )
