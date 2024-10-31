import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='pizibot_gazebo'
    descrpiton_pkg='pizibot_description'
    
    pkg_path = get_package_share_directory(package_name)

    gazebo_world_arg = DeclareLaunchArgument(
        'gazebo_world',
        default_value=os.path.join(pkg_path, 'worlds', 'world_test2.world'),
        description='Full path to the world to use for simulation'
    )
    gazebo_world = LaunchConfiguration('gazebo_world')
    
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='pizibot_',
        description='Prefix used to identify a robot when multiple instances of the same robot are present'
    )
    prefix = LaunchConfiguration('prefix')
    
    
    controllers_param_files = os.path.join(pkg_path, 'param', 'sim_controllers.yaml')    
    controllers_param_modifer = Node(package=package_name, executable='yaml_modifier_node',
                        parameters=[{'file_path': controllers_param_files},
                                    {'prefix': prefix}],
                        output='screen')
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(descrpiton_pkg),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': gazebo_world}.items()
    )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'pizibot',
                                   '-z', '0.018'],
                        
                        output='screen')
    
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{
                "base_frame_id": 'base_link',
                "left_wheel_names": ['left_wheel_joint'],
                "right_wheel_names": ['right_wheel_joint']
            }],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    twist_mux_params = os.path.join(pkg_path, 'param', 'twist_mux.yaml')
    twist_mux = Node (
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', 'diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        gazebo_world_arg,
        prefix_arg,
        controllers_param_modifer,
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        twist_mux,    
    ])

