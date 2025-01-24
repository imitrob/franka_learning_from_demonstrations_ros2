#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    height_arg = DeclareLaunchArgument('height', default_value='0.25', description='Height parameter')
    front_offset_arg = DeclareLaunchArgument('front_offset', default_value='0.4', description='Front offset parameter')
    side_offset_arg = DeclareLaunchArgument('side_offset', default_value='0.0', description='Side offset parameter')

    homing_node = Node(
        package='skills_manager',
        executable='home',
        name='homing_node',
        output='screen',
        parameters=[{
            'height': LaunchConfiguration('height'),
            'front_offset': LaunchConfiguration('front_offset'),
            'side_offset': LaunchConfiguration('side_offset'),
        }]
    )

    return LaunchDescription([
        height_arg,
        front_offset_arg,
        side_offset_arg,
        homing_node,
    ])
