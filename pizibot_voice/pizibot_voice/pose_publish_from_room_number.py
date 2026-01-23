"""
pose_publish_from_room_number.py

ROS 2 (Jazzy) node that converts a room number received on `room_number` into a navigation goal pose published
on `goal_pose`. Coordinates are loaded from data/world_test2_rooms_data.json in the package share.

Workflow:
    1. Receive room number (std_msgs/Int16) on `room_number`.
    2. Look up coordinates in world_test2_rooms_data.json.
    3. Publish PoseStamped on `goal_pose` in the map frame.

Usage:
    - Launch via voice_room_navigation.launch.py or standalone.
    - Ensure the mapping file exists and contains the requested room.

Topics:
    Subscribed: `room_number` (std_msgs/Int16)
    Published:  `goal_pose` (geometry_msgs/PoseStamped, frame_id=map)
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
# Default mapping file packaged in data/world_test2_rooms_data.json
room_data_path = os.path.join(get_package_share_directory(package_name), 'data', 'world_test2_rooms_data.json')

class PosePublishFromRoomNumber(Node):
    """
    Node that listens for a room number and publishes the corresponding goal pose for navigation.
    """
    def __init__(self):
        super().__init__('pose_publish_from_room_number')
        # Allow overriding the room mapping via ROS parameter while keeping package default.
        self.room_data_path = self.declare_parameter(
            'room_data_path', room_data_path
        ).get_parameter_value().string_value

        self.rooms_data = self.read_rooms_data(self.room_data_path)
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
        
    def read_rooms_data(self, path):
        """
        Load the room coordinates from the JSON file.
        """
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            self.get_logger().info(f"Loaded room mapping from {path}")
            return data
        except Exception as exc:  # minimal guard to fail loudly but informatively
            self.get_logger().error(f"Failed to load room mapping file {path}: {exc}")
            raise
    
def main(args=None):
    rclpy.init(args=args)
    node = PosePublishFromRoomNumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
