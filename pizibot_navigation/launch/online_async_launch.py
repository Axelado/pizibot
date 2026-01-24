"""
online_async_launch.py

This ROS 2 launch file starts the Slam Toolbox in online asynchronous mode for mapping.

The launch file configures and starts the async_slam_toolbox_node, which performs
online SLAM (Simultaneous Localization and Mapping) while the robot is moving.

Usage:
    ros2 launch pizibot_navigation online_async_launch.py

With custom parameters:
    ros2 launch pizibot_navigation online_async_launch.py slam_params_file:=/path/to/params.yaml

Author: Axel NIATO
Date: January 2026
"""

import os
import subprocess
import threading
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory


def activate_slam_after_delay():
    """Function to activate the slam_toolbox node after a delay."""
    time.sleep(3)  # Wait 3 seconds for the node to be fully started
    try:
        # First configure the node (transition id=1)
        subprocess.run([
            'ros2', 'service', 'call', '/slam_toolbox/change_state',
            'lifecycle_msgs/ChangeState', '{transition: {id: 1}}'
        ], timeout=5)
        
        time.sleep(1)  # Wait after configuration
        
        # Then activate the node (transition id=3)
        subprocess.run([
            'ros2', 'service', 'call', '/slam_toolbox/change_state',
            'lifecycle_msgs/ChangeState', '{transition: {id: 3}}'
        ], timeout=5)
    except Exception as e:
        print(f"Error activating slam_toolbox: {e}")


def generate_launch_description():
    package_name = "pizibot_navigation"
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # Use LifecycleNode for slam_toolbox
    start_async_slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ])

    # Create and start the thread to activate the node after a delay
    activation_thread = threading.Thread(target=activate_slam_after_delay, daemon=True)
    activation_thread.start()

    # EKF for sensor fusion (odometry + IMU)
    ekf_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'ekf.launch.py')
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(ekf)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
