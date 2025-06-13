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

IMPORTANT:
    You must launch the simulation (Gazebo) or start the real robot before running this launch file.
    This launch file does NOT start the robot simulation or hardware drivers.

Usage:
    ros2 launch pizibot_navigation full_mapping.launch.py

Author: Axel NIATO
Date: 11/06/2025
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import logging
from ament_index_python.packages import get_package_share_directory

# Logger configuration for info and error reporting during launch
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("pizibot_navigation.launch")

def generate_launch_description():
    """
    Generates the launch description for a full mapping (SLAM) setup for the Pizibot robot.

    This function:
        - Builds the paths to all required launch files
        - Checks their existence and logs the used paths
        - Prepares the inclusion of each launch file (SLAM, navigation, teleop, RViz)
        - Returns the global LaunchDescription
    """
    package_name = "pizibot_navigation"
    
    # Build the paths to each component's launch file
    joystick_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'joystick_teleop.launch.py')
    keyboard_teleop_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'keyboard_teleop.launch.py')

    slam_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'online_async_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')
    rviz_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rviz2.launch.py')
    
    # Path to the RViz configuration file for mapping
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'mapping.rviz')

    # Advanced logging to check the used paths
    logger.info(f"joystick_launch_path: {joystick_teleop_launch_path}")
    logger.info(f"keyboard_launch_path: {keyboard_teleop_launch_path}")
    logger.info(f"slam_launch_path: {slam_launch_path}")
    logger.info(f"navigation_launch_path: {navigation_launch_path}")
    logger.info(f"rviz_launch_path: {rviz_launch_path}")
    logger.info(f"rviz_config_file: {rviz_config_file}")
    
    # Check that all files exist, log an error if any are missing
    if not os.path.isfile(joystick_teleop_launch_path):
        logger.error(f"{joystick_teleop_launch_path} does not exist")
    if not os.path.isfile(keyboard_teleop_launch_path):
        logger.error(f"{keyboard_teleop_launch_path} does not exist")
    if not os.path.isfile(slam_launch_path):
        logger.error(f"{slam_launch_path} does not exist")
    if not os.path.isfile(navigation_launch_path):
        logger.error(f"{navigation_launch_path} does not exist")
    if not os.path.isfile(rviz_launch_path):
        logger.error(f"{rviz_launch_path} does not exist")
    if not os.path.isfile(rviz_config_file):
        logger.error(f"{rviz_config_file} does not exist")
    
    # Include each component's launch file
    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_teleop_launch_path), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    keyboard_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(keyboard_teleop_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
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
    
    # Return the complete launch description
    return LaunchDescription([
        slam,
        navigation,
        joystick_teleop,
        keyboard_teleop,
        rviz2,
    ])
