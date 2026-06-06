import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'pizibot_hardware'
    pkg_path = get_package_share_directory(package_name)

    enable_remote_ui_arg = DeclareLaunchArgument(
        'enable_remote_ui',
        default_value='false',
        description='Launch rosbridge + web_video_server + HTTP server for the web UI'
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rplidar.launch.py')
        )
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pizibot_description'), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    controller_params = os.path.join(pkg_path, 'param', 'robot_controllers.yaml')
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params],
        output='screen'
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner]
        )
    )

    twist_mux_params = os.path.join(pkg_path, 'param', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('cmd_vel_out', 'diff_drive_controller/cmd_vel')],
        output='screen'
    )

    remote_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pizibot_remote_ui'),
                'launch', 'remote_ui.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('enable_remote_ui'))
    )

    return LaunchDescription([
        enable_remote_ui_arg,
        twist_mux,
        rsp,
        delayed_controller_manager,
        joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
        lidar,
        remote_ui,
    ])
