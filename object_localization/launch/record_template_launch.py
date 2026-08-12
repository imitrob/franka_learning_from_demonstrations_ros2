#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "name_template",
            default_value="cube_template",
            description="Template object name",
        ),
        DeclareLaunchArgument(
            "overwrite_existing",
            default_value="false",
            description="Replace an existing template explicitly",
        ),
        Node(
            package="object_localization",
            executable="record_template",
            name="record_template_node",
            output="screen",
            parameters=[{
                "name_template": LaunchConfiguration("name_template"),
                "overwrite_existing": LaunchConfiguration("overwrite_existing"),
            }],
        ),
    ])
