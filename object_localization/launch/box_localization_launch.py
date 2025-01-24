#!/usr/bin/env python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Localizer node
    localizer_node = Node(
        package="object_localization",
        executable="localizer_service",
        name="localizer_node",
        output='screen',
    )

    # Active localizer node
    active_localizer_node = Node(
        package="object_localization",
        executable="active_localizer",
        name="active_localizer",
        output="screen",
    )

    # Include the camera launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("object_localization"),
                "launch",
                "camera_launch.py"
            )
        )
    )

    return LaunchDescription([
        localizer_node,
        active_localizer_node,
        camera_launch,
    ])
