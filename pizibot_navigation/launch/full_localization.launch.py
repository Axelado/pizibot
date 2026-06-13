"""
full_localization.launch.py

This ROS 2 launch file starts a complete localization setup for the Pizibot robot.
It launches:
    - Localization (AMCL, map_server)
    - The navigation stack (Nav2)
    - Joystick and keyboard teleoperation nodes
    - RViz2 with a localization-specific configuration
    - An initial pose publisher (with a delay)

Each subsystem is started via its own launch file, and the script checks for the existence of all required files before launching.
A Python logger is used to display the paths and report any missing files.

The 'robot_mode' argument (sim/real) selects sensible defaults for
'use_sim_time', 'map' and 'initial_pose', so you don't have to pass them
manually every time you switch between simulation and the real robot.
Any of these three arguments can still be overridden individually.

IMPORTANT:
    You must launch the simulation (Gazebo) or start the real robot before running this launch file.
    This launch file does NOT start the robot simulation or hardware drivers.

Usage:
    ros2 launch pizibot_navigation full_localization.launch.py
    ros2 launch pizibot_navigation full_localization.launch.py robot_mode:=real
    ros2 launch pizibot_navigation full_localization.launch.py robot_mode:=real map:=industrial-warehouse.yaml initial_pose:="[1.0, 2.0, 0.0]"

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

# Default values applied when 'use_sim_time', 'map' or 'initial_pose' are left empty,
# based on the selected 'robot_mode'.
MODE_DEFAULTS = {
    'sim': {
        'use_sim_time': 'true',
        'map': 'industrial-warehouse.yaml',
        'initial_pose': '[0.0, 0.0, 0.0]',
    },
    'real': {
        'use_sim_time': 'false',
        'map': 'home_1.yaml',
        'initial_pose': '[0.0, 0.0, -1.2566]',
    },
}


def launch_setup(context, *args, **kwargs):
    package_name = "pizibot_navigation"

    robot_mode = LaunchConfiguration('robot_mode').perform(context)
    defaults = MODE_DEFAULTS[robot_mode]

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) or defaults['use_sim_time']
    map_value = LaunchConfiguration('map').perform(context) or defaults['map']
    initial_pose = LaunchConfiguration('initial_pose').perform(context) or defaults['initial_pose']

    # Accept either a bare filename (resolved against the package's map directory)
    # or a full/relative path provided by the user.
    if os.path.sep in map_value:
        map_path = map_value
    else:
        map_path = os.path.join(get_package_share_directory(package_name), 'map', map_value)

    joystick_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'joystick_teleop.launch.py')
    keyboard_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'keyboard_teleop.launch.py')
    localization_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'localization_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')
    rviz_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rviz2.launch.py')
    publish_initial_pose_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'publish_initial_pose.launch.py')

    # Path to the RViz configuration file for localization
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'localization.rviz')

    for path in [localization_launch_path, navigation_launch_path, rviz_launch_path, rviz_config_file, publish_initial_pose_launch_path]:
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
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map_path}.items()
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

    # The initial pose publisher waits for AMCL to subscribe to /initialpose
    # before publishing, so no startup delay is needed here.
    publish_initial_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(publish_initial_pose_launch_path),
        launch_arguments={'use_sim_time': use_sim_time, 'initial_pose': initial_pose}.items()
    )

    return [
        localization,
        navigation,
        joystick_teleop,
        keyboard_teleop,
        rviz2,
        publish_initial_pose,
    ]


def generate_launch_description():
    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode', default_value='sim', choices=['sim', 'real'],
        description="sim or real: selects the default 'use_sim_time', 'map' and 'initial_pose' values"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='',
        description="Use simulation clock if true. Empty: derived from 'robot_mode'"
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description="Map file to use for localization (filename in the package's map directory, or full path). Empty: derived from 'robot_mode'"
    )
    initial_pose_arg = DeclareLaunchArgument(
        'initial_pose', default_value='',
        description="Initial pose of the robot as '[x, y, yaw]'. Empty: derived from 'robot_mode'"
    )

    return LaunchDescription([
        robot_mode_arg,
        use_sim_time_arg,
        map_arg,
        initial_pose_arg,
        OpaqueFunction(function=launch_setup),
    ])
