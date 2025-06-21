"""
voice_room_navigation.launch.py

This ROS 2 launch file starts the voice command stack for room navigation on the Pizibot robot.

**IMPORTANT:**  
This voice system is limited to sending the robot to a specific room.  
It does not support general voice control or arbitrary commands.

Launched nodes:
    - keyboard_activator: Activates the voice system via keyboard input.
    - voice_recorder: Records and processes the user's voice command.
    - pose_publish_from_room_number: Publishes the navigation goal corresponding to the recognized room number.

Usage:
    ros2 launch pizibot_voice voice_room_navigation.launch.py

Author: Axel NIATO
Date: 20/06/2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'pizibot_voice'
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    keyboard_activator = Node(
        package=package_name,
        executable='keyboard_activator',
        name='keyboard_activator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    voice_recorder = Node(
        package=package_name,
        executable='voice_recorder',
        name='voice_recorder',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    pose_publish_from_room_number = Node(
        package=package_name,
        executable='pose_publish_from_room_number',
        name='pose_publish_from_room_number',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        keyboard_activator,
        voice_recorder,
        pose_publish_from_room_number,
    ])