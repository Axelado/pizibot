"""
ekf.launch.py

Launch file for the Extended Kalman Filter (EKF) node from robot_localization.
This node fuses wheel odometry and IMU data for improved localization accuracy.

The EKF provides:
    - Smoother odometry estimates
    - Better heading estimation using IMU gyroscope
    - Resilience to wheel slip using IMU accelerometer

Usage:
    ros2 launch pizibot_navigation ekf.launch.py

Author: Axel NIATO
Date: January 2026
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'pizibot_navigation'
    
    # Get paths
    pkg_share = get_package_share_directory(package_name)
    ekf_params_file = os.path.join(pkg_share, 'params', 'ekf.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/odom/filtered')
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        ekf_node
    ])
