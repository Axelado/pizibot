"""
voice_room_navigation.launch.py

Launches the voice command pipeline for room navigation on Pizibot (ROS 2 Jazzy).

Nodes started:
    - keyboard_activator: toggles recording with the space bar (publishes start_talking)
    - voice_recorder: records audio, runs speech-to-text, publishes room_number
    - pose_publish_from_room_number: maps room_number to goal_pose

Limitations: only handles simple "go to room N" commands; not a general voice assistant.

Usage:
    ros2 launch pizibot_voice voice_room_navigation.launch.py [use_sim_time:=true|false]
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'pizibot_voice'
    default_room_data_path = os.path.join(
        get_package_share_directory(package_name), 'data', 'world_test2_rooms_data.json'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    room_data_path_arg = DeclareLaunchArgument(
        'room_data_path',
        default_value=default_room_data_path,
        description='Path to the room mapping JSON file'
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
        parameters=[
            {'use_sim_time': use_sim_time},
            {'room_data_path': LaunchConfiguration('room_data_path')}
        ]
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        room_data_path_arg,
        keyboard_activator,
        voice_recorder,
        pose_publish_from_room_number,
    ])