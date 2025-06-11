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