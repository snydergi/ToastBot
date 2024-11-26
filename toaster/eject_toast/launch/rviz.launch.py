from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

import xacro

def generate_launch_description():
    # Path to URDF file
    pkg_path = FindPackageShare('eject_toast').find('eject_toast')
    default_model_path = os.path.join(pkg_path, 'urdf/toaster.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_path, 'config/toast_config.rviz')

    robot_desc = xacro.process_file(default_model_path).toxml()
    robot_description = {'robot_description': robot_desc}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'toaster_body']
    )

    static_transform_toast_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_toast_map',
        arguments=['0', '0', '0.15', '0', '0', '0', 'map', 'toast']
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    ejector_node = Node(
        package='eject_toast',
        executable='eject_toast_node',
        name='eject_toast_node',
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path,
                              description='Path to robot URDF file'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path,
                              description='Path to RViz config file'),
        robot_state_publisher_node,
        rviz_node,
        static_transform_node,
        static_transform_toast_map,
        ejector_node
    ])
