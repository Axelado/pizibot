"""
pose_publish_from_room_number.py

This ROS 2 node listens for a room number on the 'room_number' topic and publishes the corresponding goal pose
on the 'goal_pose' topic for navigation. The mapping between room numbers and their coordinates is loaded from
a JSON file (rooms_data.json) located in the package's data directory.

Workflow:
    1. Receives a room number (Int16) on the 'room_number' topic.
    2. Looks up the corresponding coordinates in rooms_data.json.
    3. Publishes a PoseStamped message with the goal pose on the 'goal_pose' topic.

Usage:
    - Run as part of the voice_room_navigation.launch.py launch file or standalone.
    - Make sure the rooms_data.json file exists and is properly formatted.

Author: Axel NIATO
Date: 20/06/2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header

import json

package_name = "pizibot_voice"
room_data_path = os.path.join(get_package_share_directory(package_name), 'data','rooms_data.json')

class PosePublishFromRoomNumber(Node):
    """
    Node that listens for a room number and publishes the corresponding goal pose for navigation.
    """
    def __init__(self):
        super().__init__('pose_publish_from_room_number')
        self.rooms_data = self.read_rooms_data()
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.subscription = self.create_subscription(
            Int16,
            'room_number',  
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info("pose_publish_from_room_number has started")
        
    def listener_callback(self, msg):
        """
        Callback when a room number is received.
        """
        self.get_logger().info("Room number received")
        room_number = msg.data
        point = self.select_room(room_number)
        if point is not None: 
            self.publish_pose(point)
        
    def select_room(self, room_number):
        """
        Select the coordinates for the given room number from the JSON data.
        """
        try:
            point = self.rooms_data[str(room_number)]
            self.get_logger().info(f"Room {room_number} is at position: {point}")
            return point
        except KeyError:
            self.get_logger().info(f"Room {room_number} does not exist")
            return None
        
    def publish_pose(self, point):
        """
        Publish the goal pose corresponding to the selected room.
        """
        X = float(point['x'])
        Y = float(point['y'])
        
        msg = PoseStamped()
        # Set the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.header.frame_id = 'map'
        
        # Set the pose
        msg.pose.position = Point(x=X, y=Y, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pose_publisher_.publish(msg)
        self.get_logger().info("Goal pose published")
        
    def read_rooms_data(self):
        """
        Load the room coordinates from the JSON file.
        """
        with open(room_data_path, 'r') as f:
            return json.load(f)
    
def main(args=None):
    rclpy.init(args=args)
    node = PosePublishFromRoomNumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
