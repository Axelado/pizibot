"""
publish_initial_pose.launch.py

This ROS 2 launch file starts the initial_pose_publisher node, which publishes
an initial pose estimate to the /initialpose topic for AMCL localization.

The node publishes the pose for 3 seconds at 10 Hz to ensure AMCL receives it.

Usage:
    ros2 launch pizibot_navigation publish_initial_pose.launch.py initial_pose:="[x, y, yaw]"

Example:
    ros2 launch pizibot_navigation publish_initial_pose.launch.py initial_pose:="[1.0, 2.0, 0.0]"

Author: Axel NIATO
Date: January 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the initial_pose argument as [x, y, yaw]
    declare_initial_pose = DeclareLaunchArgument(
        'initial_pose',
        default_value='[0.0, 0.0, 0.0]',
        description='Initial pose of the robot as [x, y, yaw] (meters, radians)'
    )

    # Node to publish the initial pose
    initial_pose_publisher_node = Node(
        package='pizibot_navigation',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{'initial_pose': LaunchConfiguration('initial_pose')}]
    )

    return LaunchDescription([
        declare_initial_pose,
        initial_pose_publisher_node
    ])