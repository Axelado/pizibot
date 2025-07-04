import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    
    # Prefix argument for robot identification
    prefix = LaunchConfiguration('prefix')
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Prefix used to identify a robot when multiple instances of the same robot are present'
    )

    pkg_path = os.path.join(get_package_share_directory('pizibot_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'pizibot.urdf.xacro')

    # Generate the robot description using xacro and the prefix argument
    robot_description_config = Command(['xacro ', xacro_file, ' prefix:=' , prefix, ' sim_mode:=', use_sim_time])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        use_sim_time_arg,
        prefix_arg,
        node_robot_state_publisher,
    ])
