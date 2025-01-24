#!/usr/bin/env python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
def generate_launch_description():
    color_profile = DeclareLaunchArgument("rgb_camera.color_profile", default_value="848,480,3")
    log_level = DeclareLaunchArgument("log_level", default_value="error")
    set_lrs_log_level = SetEnvironmentVariable("LRS_LOG_LEVEL", "none")

    # Include the Realsense camera launch file with resolution parameters
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py"
            )
        ),
        launch_arguments={
            "rgb_camera.color_profile": "848,480,30",
            "log_level": "error",
        }.items()
    )

    # Static transform publisher node
    camera_tf_publisher_node = Node(
        package="object_localization",
        executable="static_transform_camera",
        name="camera_tf_publisher",
        output="screen",
    )

    return LaunchDescription([
        color_profile,
        log_level,
        set_lrs_log_level,
        realsense_launch,
        camera_tf_publisher_node
    ])
