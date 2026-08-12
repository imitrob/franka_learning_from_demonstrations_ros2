#!/usr/bin/env python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # /scene is published whenever something is subscribed to it, so watching the
    # scene needs no argument here -- just open a consumer (the gestures
    # dashboard, or `ros2 topic echo /scene`). These two only tune it.
    scene_rate = DeclareLaunchArgument(
        "scene_rate", default_value="1.0",
        description="How often to recompute /scene, in Hz. One get_scene pass "
                    "costs roughly 130 ms + 30 ms per saved template, so rates "
                    "above ~3 Hz will not be met with several templates.")
    scene_home_tolerance = DeclareLaunchArgument(
        "scene_home_tolerance", default_value="0.05",
        description="How far (m) the end effector may sit from the template "
                    "capture pose before matches stop being believed. Beyond it "
                    "the last seen poses are republished with their original "
                    "stamp instead of the objects disappearing.")

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
        parameters=[{
            "scene_rate": ParameterValue(
                LaunchConfiguration("scene_rate"), value_type=float),
            "scene_home_tolerance": ParameterValue(
                LaunchConfiguration("scene_home_tolerance"), value_type=float),
        }],
    )

    template_recorder_node = Node(
        package="object_localization",
        executable="template_recorder_server",
        name="template_recorder_server",
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
        scene_rate,
        scene_home_tolerance,
        localizer_node,
        active_localizer_node,
        template_recorder_node,
        camera_launch,
    ])
