from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates a ROS 2 launch description for starting the teleop_twist_keyboard_for_azerty node.
    This function creates a launch description that launches the teleop_twist_keyboard_for_azerty node
    in a new xterm terminal. The node is configured to remap the '/cmd_vel' topic to '/cmd_vel_key' and
    outputs logs to the screen.
    Returns:
        LaunchDescription: A ROS 2 LaunchDescription object containing the teleop keyboard node.
    # The teleop_twist_keyboard_for_azerty node allows keyboard-based teleoperation using an AZERTY keyboard layout.
    """
    
    teleop_keyboard_node = Node(
            package='teleop_twist_keyboard_for_azerty',
            executable='teleop_twist_keyboard_for_azerty',
            remappings=[('/cmd_vel', '/cmd_vel_key')],
            output='screen',
            
            prefix='xterm -e' 
         )

    return LaunchDescription([
        teleop_keyboard_node,
    ])
