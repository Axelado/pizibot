"""
full_mapping.launch.py

This ROS 2 launch file starts a complete mapping (SLAM) setup for the Pizibot robot.
It launches:
    - SLAM (online mapping)
    - The navigation stack (Nav2)
    - Joystick and keyboard teleoperation nodes
    - RViz2 with a mapping-specific configuration

Each subsystem is started via its own launch file, and the script checks for the existence of all required files before launching.
A Python logger is used to display the paths and report any missing files.

The 'robot_mode' argument (sim/real) selects the default 'use_sim_time' value,
so you don't have to pass it manually every time you switch between simulation
and the real robot. It can still be overridden individually.

IMPORTANT:
    You must launch the simulation (Gazebo) or start the real robot before running this launch file.
    This launch file does NOT start the robot simulation or hardware drivers.

Usage:
    ros2 launch pizibot_navigation full_mapping.launch.py
    ros2 launch pizibot_navigation full_mapping.launch.py robot_mode:=real

Author: Axel NIATO
Date: January 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os
import logging
from ament_index_python.packages import get_package_share_directory

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("pizibot_navigation.launch")

# Default 'use_sim_time' value applied when left empty, based on the selected 'robot_mode'.
MODE_DEFAULTS = {
    'sim': {'use_sim_time': 'true'},
    'real': {'use_sim_time': 'false'},
}


def launch_setup(context, *args, **kwargs):
    package_name = "pizibot_navigation"

    robot_mode = LaunchConfiguration('robot_mode').perform(context)
    defaults = MODE_DEFAULTS[robot_mode]

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) or defaults['use_sim_time']

    joystick_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'joystick_teleop.launch.py')
    keyboard_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'keyboard_teleop.launch.py')
    slam_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'online_async_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')
    rviz_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rviz2.launch.py')
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'mapping.rviz')

    for path in [slam_launch_path, navigation_launch_path, rviz_launch_path, rviz_config_file]:
        if not os.path.isfile(path):
            logger.error(f"{path} does not exist")

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_teleop_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    keyboard_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(keyboard_teleop_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={'use_sim_time': use_sim_time,
                          'map_subscribe_transient_local': 'true'}.items()
    )
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path),
        launch_arguments={'use_sim_time': use_sim_time,
                          'rviz_config_file': rviz_config_file}.items()
    )

    return [
        slam,
        # navigation,
        joystick_teleop,
        keyboard_teleop,
        rviz2,
    ]


def generate_launch_description():
    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode', default_value='sim', choices=['sim', 'real'],
        description="sim or real: selects the default 'use_sim_time' value"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='',
        description="Use simulation clock if true. Empty: derived from 'robot_mode'"
    )

    return LaunchDescription([
        robot_mode_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup),
    ])
