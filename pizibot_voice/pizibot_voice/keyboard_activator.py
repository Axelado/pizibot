"""
keyboard_activator.py

This ROS 2 node allows the user to activate or deactivate the voice command system using the space bar.
When the space bar is pressed, the node publishes '1' on the 'start_talking' topic to signal the start of voice recording.
When the space bar is released, it publishes '0' to signal the end of voice recording.

Usage:
    - Run this node as part of the voice_room_navigation.launch.py launch file or standalone.
    - Press and hold the space bar to activate voice command.
    - Release the space bar to deactivate voice command.

Author: Axel NIATO
Date: 20/06/2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardActivator(Node):
    def __init__(self):
        super().__init__('keyboard_activator')
        self.publisher_ = self.create_publisher(String, 'start_talking', 10)
        self.get_logger().info('Keyboard Activator Node has been started.')
        self.key_pressed = False
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        if key == keyboard.Key.space:
            if not self.key_pressed:
                self.key_pressed = True
                msg = String()
                msg.data = '1'
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: "{msg.data}"')

    def on_release(self, key):
        if key == keyboard.Key.space:
            self.key_pressed = False
            msg = String()
            msg.data = '0'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardActivator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Activator Node is stopping.')
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
