#!/usr/bin/env python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# CAMERA_SERIAL_NO = '"105322250337"'
CAMERA_SERIAL_NO = '"318122300789"'

def generate_launch_description():
    log_level = DeclareLaunchArgument("log_level", default_value="warn")
    serial_no = DeclareLaunchArgument("serial_no", default_value=CAMERA_SERIAL_NO)

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
            "rgb_camera.profile": "1280,720,30",
            "depth_module.profile": "848,480,30",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "log_level": "info",
            "initial_reset": "true",
            "serial_no": LaunchConfiguration("serial_no"),
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
        log_level,
        serial_no,
        realsense_launch,
        camera_tf_publisher_node
    ])
