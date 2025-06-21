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

IMPORTANT:
    You must launch the simulation (Gazebo) or start the real robot before running this launch file.
    This launch file does NOT start the robot simulation or hardware drivers.

Usage:
    ros2 launch pizibot_navigation full_localization.launch.py

Author: Axel NIATO
Date: 11/06/2025
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import logging
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

# Logger configuration for info and error reporting during launch
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("pizibot_navigation.launch")

def generate_launch_description():
    package_name = "pizibot_navigation"
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory(package_name), 'map', 'world_test2.yaml'),
        description='Full path to the map to use for localization'
    )
    
    map = LaunchConfiguration('map')
    
    # Build the paths to each component's launch file
    joystick_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'joystick_teleop.launch.py')
    keyboard_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'keyboard_teleop.launch.py')
    localization_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'localization_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')
    rviz_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rviz2.launch.py')
    publish_initial_pose_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'publish_initial_pose.launch.py')
    
    # Path to the RViz configuration file for localization
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'localization.rviz')

    # Log the paths to check their correctness
    logger.debug(f"joystick_launch_path: {joystick_teleop_launch_path}")
    logger.debug(f"keyboard_launch_path: {keyboard_teleop_launch_path}")
    logger.debug(f"localization_launch_path: {localization_launch_path}")
    logger.debug(f"navigation_launch_path: {navigation_launch_path}")
    logger.debug(f"rviz_launch_path: {rviz_launch_path}")
    logger.debug(f"rviz_config_file: {rviz_config_file}")
    logger.debug(f"publish_initial_pose_launch_path: {publish_initial_pose_launch_path}")

    # Check that all files exist, log an error if any are missing
    if not os.path.isfile(localization_launch_path):
        logger.error(f"{localization_launch_path} does not exist")
    if not os.path.isfile(navigation_launch_path):
        logger.error(f"{navigation_launch_path} does not exist")
    if not os.path.isfile(rviz_launch_path):
        logger.error(f"{rviz_launch_path} does not exist")
    if not os.path.isfile(rviz_config_file):
        logger.error(f"{rviz_config_file} does not exist")
    if not os.path.isfile(publish_initial_pose_launch_path):
        logger.error(f"{publish_initial_pose_launch_path} does not exist")
    
    # Include each component's launch file
    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_teleop_launch_path), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    keyboard_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(keyboard_teleop_launch_path), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path), 
        launch_arguments={'use_sim_time': 'true',
                          'map' : map}.items()
    )
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path), 
        launch_arguments={'use_sim_time': 'true',
                          'map_subscribe_trasient_local': 'true'}.items()
    )
    
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path), 
        launch_arguments={'use_sim_time': 'true',
                          'rviz_config_file': rviz_config_file}.items()
    )
    
    # Delay the initial pose publisher by 10 seconds after simulation starts
    publish_initial_pose = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(publish_initial_pose_launch_path),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        navigation,
        localization,
        joystick_teleop,
        keyboard_teleop,
        rviz2,
        publish_initial_pose
    ])
