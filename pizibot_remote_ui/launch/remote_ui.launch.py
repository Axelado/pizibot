import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('pizibot_remote_ui')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='ROS topic image source for web_video_server'
    )
    image_topic = LaunchConfiguration('image_topic')

    rosbridge_params = os.path.join(pkg_path, 'params', 'rosbridge.yaml')

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[rosbridge_params],
        output='screen'
    )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080, 'ros_threads': 4}],
        output='screen'
    )

    # realpath resolves symlinks (colcon --symlink-install) → source tree
    _src_pkg = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    _src_dist = os.path.join(_src_pkg, 'web', 'dist')
    # Fall back to colcon-installed copy if source dist is missing
    web_dist_path = _src_dist if os.path.isdir(os.path.join(_src_dist, 'assets')) else os.path.join(pkg_path, 'web')
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '3000', '--directory', web_dist_path],
        output='screen',
        name='web_http_server'
    )

    return LaunchDescription([
        image_topic_arg,
        rosbridge,
        web_video_server,
        http_server,
    ])
