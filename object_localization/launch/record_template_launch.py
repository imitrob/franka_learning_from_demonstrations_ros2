from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from panda_control.home_pose import HOME_POSE


def launch_setup(context, *args, **kwargs):
    """Home first, then record. The arm has to be out of the camera's way
    before the template is grabbed, which used to be a separate
    `ros2 launch skills_manager home_launch.py` the user had to remember."""
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

    if LaunchConfiguration('homing').perform(context).lower() not in ('true', '1'):
        return [template_node, shutdown_on_record_exit]

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
                                           on_exit=[template_node])),
        shutdown_on_record_exit,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'name_template',
            default_value='cube_template',
            description='Template of the object to search during localization'
        ),
        DeclareLaunchArgument(
            'homing', default_value='true',
            description='Home the robot before recording the template'
        ),
        OpaqueFunction(function=launch_setup),
    ])
