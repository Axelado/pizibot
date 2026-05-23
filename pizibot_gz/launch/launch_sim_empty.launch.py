import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """Launch Pizibot simulation in empty world with Gazebo Harmonic."""

    package_name = 'pizibot_gz'
    description_pkg = 'pizibot_description'
    
    pkg_path = get_package_share_directory(package_name)
    description_path = get_package_share_directory(description_pkg)
    

    # Arguments
    gazebo_world_arg = DeclareLaunchArgument(
        'gazebo_world',
        default_value=os.path.join(pkg_path, 'worlds', 'world_minimal.sdf'),
        description='Full path to the world SDF file to use for simulation'
    )
    gazebo_world = LaunchConfiguration('gazebo_world')

    # Set Gazebo resource paths for mesh resolution
    # We must add the parent 'share' directory (install/share) so package://pizibot_description/meshes/... resolves.
    description_share_parent = os.path.dirname(description_path)
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_RESOURCE_PATH',
        description_share_parent
    )
    # Also set GZ_SIM_RESOURCE_PATH for compatibility with some gz versions
    set_gz_sim_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        description_share_parent
    )
    # Robot description command
    robot_description_command = Command([
        'xacro', ' ',
        os.path.join(description_path, 'urdf', 'pizibot.urdf.xacro'), ' ',
        'sim_mode:=true'
    ])

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_command, value_type=str),
            'use_sim_time': True,
            'publish_frequency': 50.0
        }],
        output='screen'
    )

    # Gazebo Harmonic
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', gazebo_world, '-v', '4'],
        output='screen'
    )

    # Spawn the robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pizibot',
            '-z', '0.02'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_gz_sim_resource_path,
        gazebo_world_arg,
        rsp,
        gz_sim,
        spawn_entity,
    ])
