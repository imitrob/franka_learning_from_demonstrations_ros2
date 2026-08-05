#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    home_tolerance = DeclareLaunchArgument(
        "home_tolerance",
        default_value="0.05",
        description="Cartesian distance from HOME_POSE that counts as home (m)",
    )
    home_orientation_tolerance = DeclareLaunchArgument(
        "home_orientation_tolerance",
        default_value="0.1",
        description="Angular distance from HOME_POSE that counts as home (rad)",
    )
    server = Node(
        package="skills_manager",
        executable="lfd_server",
        name="lfd_server",
        output="screen",
        parameters=[{
            "home_tolerance": LaunchConfiguration("home_tolerance"),
            "home_orientation_tolerance": LaunchConfiguration(
                "home_orientation_tolerance"
            ),
        }],
    )
    return LaunchDescription([home_tolerance, home_orientation_tolerance, server])
