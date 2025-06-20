#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    name_skill_arg = DeclareLaunchArgument(
        'name_skill',
        default_value='skill',
        description='Name of the skill to record'
    )
    name_skill_arg = DeclareLaunchArgument(
        'name_template',
        default_value='template',
        description='Name of the template to localize'
    )


    recording_node = Node(
        package='skills_manager',
        executable='record_skill',
        name='recording_node',
        output='screen',
        parameters=[{
            'name_skill': LaunchConfiguration('name_skill'),
            'name_template': LaunchConfiguration('name_template'),
        }],
    )

    return LaunchDescription([
        name_skill_arg,
        recording_node,
    ])
