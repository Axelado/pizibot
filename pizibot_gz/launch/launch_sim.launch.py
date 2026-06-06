import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, AppendEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch Pizibot simulation with Gazebo Harmonic (gz-sim)."""

    package_name = 'pizibot_gz'
    description_pkg = 'pizibot_description'
    
    pkg_path = get_package_share_directory(package_name)

    # Arguments
    enable_remote_ui_arg = DeclareLaunchArgument(
        'enable_remote_ui',
        default_value='false',
        description='Launch rosbridge + web_video_server + HTTP server for the web UI'
    )

    gazebo_world_arg = DeclareLaunchArgument(
        'gazebo_world',
        default_value=os.path.join(pkg_path, 'worlds', 'industrial-warehouse.sdf'),
        description='Full path to the world SDF file to use for simulation'
    )
    gazebo_world = LaunchConfiguration('gazebo_world')

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value ='0.0',
        description='robot x coordinate'
    )
    x_pose = LaunchConfiguration('x_pose') 
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value ='0.0',
        description='robot y coordinate'
    )
    y_pose = LaunchConfiguration('y_pose') 
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value ='0.08',
        description='robot z coordinate'
    )
    z_pose = LaunchConfiguration('z_pose') 
    
    
    description_path = get_package_share_directory(description_pkg)

    # Set Gazebo resource paths for mesh resolution
    description_share_parent = os.path.dirname(description_path)
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_RESOURCE_PATH',
        description_share_parent
    )
    set_gz_sim_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        description_share_parent
    )

    # Robot description command
    robot_description_cmd = Command([
        'xacro', ' ',
        os.path.join(description_path, 'urdf', 'pizibot.urdf.xacro'), ' ',
        'sim_mode:=true'
    ])

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_cmd, value_type=str),
            'use_sim_time': True,
            'publish_frequency': 50.0
        }],
        output='screen'
    )

    # Gazebo Harmonic (gz-sim) - using ExecuteProcess
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', gazebo_world, '-v', '4'],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pizibot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )

    # Load ROS 2 <-> Gazebo bridge configuration
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'pizibot_gz_bridge.yaml'
    )

    # Twist mux parameters
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'twist_mux.yaml'
    )

    # Start ROS 2 <-> Gazebo topic bridge
    gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Bridge camera image topic (RGB camera)
    gazebo_ros_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )
    
    # Bridge semantic segmentation camera - labels map
    # Provides pixel-wise semantic labels for scene understanding
    gazebo_ros_labels_map_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/semantic/labels_map'],
        output='screen',
    )
    
    # Bridge semantic segmentation camera - colored visualization
    # Provides color-coded visualization of semantic labels
    gazebo_ros_colored_map_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/semantic/colored_map'],
        output='screen',
    )

    # Start twist_mux to prioritize cmd_vel sources
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_params],
        output='screen',
    )

    # Convert TwistStamped (twist_mux output) to Twist for the Gazebo bridge
    twist_stamped_to_twist = Node(
        package='pizibot_gz',
        executable='twist_stamped_to_twist.py',
        name='twist_stamped_to_twist',
        output='screen',
    )

    remote_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pizibot_remote_ui'),
                'launch', 'remote_ui.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('enable_remote_ui'))
    )

    return LaunchDescription([
        enable_remote_ui_arg,
        gazebo_world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        set_gz_resource_path,
        set_gz_sim_resource_path,
        rsp,
        gz_sim,
        spawn_entity,
        gazebo_ros_bridge,
        gazebo_ros_image_bridge,
        gazebo_ros_labels_map_bridge,
        gazebo_ros_colored_map_bridge,
        twist_mux,
        twist_stamped_to_twist,
        remote_ui,
    ])
