from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    template_name_arg = DeclareLaunchArgument(
        'name_template',
        default_value='cube_template',
        description='Template of the object to search during localization'
    )

    template_node = Node(
        package='object_localization',
        executable='record_template',
        name='record_template_node',
        output='screen',
        parameters=[{'name_template': LaunchConfiguration('name_template')}]
    )

    shutdown_on_record_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=template_node,
            on_exit=[EmitEvent(event=Shutdown(reason='record_template finished'))]
        )
    )

    # Return the launch description
    return LaunchDescription([
        template_name_arg,
        template_node,
        shutdown_on_record_exit
    ])
