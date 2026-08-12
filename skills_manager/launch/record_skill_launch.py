#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    recording_node = Node(
        package='skills_manager',
        executable='record_skill',
        name='recording_node',
        output='screen',
        parameters=[{
            'name_skill': LaunchConfiguration('name_skill'),
            'name_template': LaunchConfiguration('name_template'),
            'homing': LaunchConfiguration('homing'),
            'overwrite_existing': LaunchConfiguration('overwrite_existing'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'name_skill', default_value='skill',
            description='Name of the skill to record'
        ),
        DeclareLaunchArgument(
            'name_template', default_value='',
            description='Optional template override; derived from name_skill when empty'
        ),
        DeclareLaunchArgument(
            'homing', default_value='true',
            description='Home the robot before recording'
        ),
        DeclareLaunchArgument(
            'overwrite_existing', default_value='false',
            description='Replace an existing demonstration explicitly'
        ),
        recording_node,
    ])
