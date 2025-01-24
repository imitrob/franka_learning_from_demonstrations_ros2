from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    color_profile = DeclareLaunchArgument("rgb_camera.color_profile", default_value="848,480,3")

    # Declare arguments
    template_name_arg = DeclareLaunchArgument(
        'template_name', 
        default_value='cube_template', 
        description='Template of the object to search during localization'
    )

    # Include the rs_camera.launch.py from the realsense2_camera package
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            "rgb_camera.color_profile": "848,480,30",
        }.items()
    )

    # Define the node
    template_node = Node(
        package='object_localization',
        executable='record_template',
        name='record_template_node',
        output='screen',
        parameters=[{'template_name': LaunchConfiguration('template_name')}]
    )

    # Return the launch description
    return LaunchDescription([
        template_name_arg,
        color_profile,
        realsense_launch,
        template_node
    ])
