#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from panda_control.home_pose import HOME_POSE


def launch_setup(context, *args, **kwargs):
    """Home first, then record. Recording a localized action starts from the
    home pose, so homing used to be a separate `home_launch.py` run before
    this one."""
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

    shutdown_on_record_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=recording_node,
            on_exit=[EmitEvent(event=Shutdown(reason='record_skill finished'))]
        )
    )

    if LaunchConfiguration('homing').perform(context).lower() not in ('true', '1'):
        return [recording_node, shutdown_on_record_exit]

    homing_node = Node(
        package='skills_manager',
        executable='home',
        name='homing_node',
        output='screen',
        parameters=[{
            'height': float(HOME_POSE.position[2]),
            'front_offset': float(HOME_POSE.position[0]),
            'side_offset': float(HOME_POSE.position[1]),
        }]
    )
    # The homing node holds the robot until it exits; recording starts after.
    return [
        homing_node,
        RegisterEventHandler(OnProcessExit(target_action=homing_node,
                                           on_exit=[recording_node])),
        shutdown_on_record_exit,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'name_skill', default_value='skill',
            description='Name of the skill to record'
        ),
        DeclareLaunchArgument(
            'name_template', default_value='template',
            description='Name of the template to localize'
        ),
        DeclareLaunchArgument(
            'homing', default_value='true',
            description='Home the robot before recording'
        ),
        OpaqueFunction(function=launch_setup),
    ])
