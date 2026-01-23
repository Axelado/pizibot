"""
Camera Publisher Launch File for Pizibot Vision Package

This launch file starts the camera publisher node that captures images
from a connected USB camera and publishes them to a ROS 2 topic.

The node supports configurable camera parameters including device index,
resolution, frame rate, and topic name. Default values can be overridden
at launch time.

Usage Examples:
    # Launch with default parameters (640x480 at 30 FPS on camera 0)
    ros2 launch pizibot_vision camera.launch.py
    
    # Override specific parameters
    ros2 launch pizibot_vision camera.launch.py camera_index:=1 fps:=15
    
    # Custom resolution and topic
    ros2 launch pizibot_vision camera.launch.py \\
        width:=1280 height:=720 image_topic:="/front_camera/image"

Parameters:
    - camera_index (int): Camera device index to use (0-9)
    - image_topic (str): ROS topic name for image publishing
    - width (int): Image width in pixels
    - height (int): Image height in pixels
    - fps (int): Capture and publish frame rate
    - frame_id (str): TF frame ID for image header

Published Topics:
    - camera/image_raw (sensor_msgs/Image): Raw camera frames in BGR8 format

Dependencies:
    - pizibot_vision: Camera publisher node package
    - ROS 2 Jazzy

Author: Axel NIATO
License: Apache-2.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for the camera publisher node.
    
    Creates and configures the camera_publisher_node with parameters
    that can be overridden at launch time.
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    # Get the package share directory for accessing config files
    pkg_share = get_package_share_directory('pizibot_vision')
    
    # Path to default parameter configuration file
    default_config = os.path.join(pkg_share, 'config', 'camera_params.yaml')
    
    # ==============================================================================
    # Launch Arguments - Configurable Parameters
    # ==============================================================================
    # Each argument can be overridden at launch time via camera_index:=1 syntax
    
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Index of the camera device to use (0-9). Auto-fallback to available devices.'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='camera/image_raw',
        description='ROS 2 topic name for publishing camera images'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Capture image width in pixels'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Capture image height in pixels'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frame capture rate in frames per second'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='TF (Transform) frame ID for the camera in the image header'
    )
    
    # ==============================================================================
    # Node Configuration - camera_publisher_node
    # ==============================================================================
    # Instantiate the camera publisher node with configured parameters
    
    camera_publisher_node = Node(
        package='pizibot_vision',
        executable='camera_publisher_node',
        name='camera_publisher',
        output='screen',  # Print node output to console
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'image_topic': LaunchConfiguration('image_topic'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )
    
    # ==============================================================================
    # Launch Description Assembly
    # ==============================================================================
    # Combine all launch arguments and nodes into the final launch description
    
    return LaunchDescription([
        camera_index_arg,
        image_topic_arg,
        width_arg,
        height_arg,
        fps_arg,
        frame_id_arg,
        camera_publisher_node,
    ])
