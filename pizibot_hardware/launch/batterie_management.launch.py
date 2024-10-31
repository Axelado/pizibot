from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    package_name='pizibot_hardware'    
    
    battery_management = Node(
            name='battery_management',
            package=package_name,
            executable='battery_management_node',
            output='screen',
    )
    
    battery_state_publisher = Node(
            name='battery_state_publisher',
            package=package_name,
            executable='battery_state_publisher_node',
            output='screen',
    )
    delayed_battery_state_publisher = TimerAction(period=5.0,actions=[battery_state_publisher])

    return LaunchDescription([        
        battery_management,
        delayed_battery_state_publisher,
    ])