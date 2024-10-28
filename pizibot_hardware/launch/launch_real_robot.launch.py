import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    package_name='pizibot_hardware'
    pkg_path = get_package_share_directory(package_name)
    
    lidar_launch_path = os.path.join(pkg_path, 'launch', 'rplidar.launch.py')
    
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('pizibot_description'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])


    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='pizibot_',
        description='Prefix used to identify a robot when multiple instances of the same robot are present'
    )
    prefix = LaunchConfiguration('prefix')
    
    controller_params = os.path.join(pkg_path, 'param', 'robot_controllers.yaml')
    controllers_param_modifer = Node(package=package_name, executable='yaml_modifier_node',
                        parameters=[{'file_path': controller_params},
                                    {'prefix': prefix}],
                        output='screen')
    
    controller_params_generated = os.path.join(pkg_path, 'param', 'robot_controllers_generated.yaml')
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                controller_params_generated],
        )  
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
    twist_mux_params = os.path.join(pkg_path, 'param', 'twist_mux.yaml')
    twist_mux = Node(
            package='custom_twist_mux',
            executable='twist_mux',
            name='custom_twist_mux',
            parameters=[twist_mux_params]
    )

    return LaunchDescription([
        prefix_arg,
        controllers_param_modifer,
        twist_mux,
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        lidar,
    ])