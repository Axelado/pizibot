#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

esp32_hostname_or_ip = "192.168.1.30"
class StreamGetter(Node):
    def __init__(self):
        super().__init__('stream_getter')
        self.get_logger().info("Node 'stream_getter' launched.")
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.cap = cv2.VideoCapture(f"http://{esp32_hostname_or_ip}:81/stream")
        self.bridge = CvBridge()

        # Actively wait for the stream to start (max 10s)
        timeout = 10
        start_time = time.time()
        while not self.cap.isOpened() and (time.time() - start_time) < timeout:
            self.get_logger().info("Waiting for ESP32-CAM stream to start...")
            time.sleep(0.5)
            self.cap.open(f"http://{esp32_hostname_or_ip}:81/stream")
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video stream at {esp32_hostname_or_ip}")
        else:
            self.get_logger().info(f"Video stream detected at {esp32_hostname_or_ip}")
            
        self.timer = self.create_timer(0.1, self.timer_callback)
        

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().error("Video stream is not opened.")
            self.cap.open(f"http://{esp32_hostname_or_ip}:81/stream")
            self.get_logger().info("Attempting to reopen the stream...")
            time.sleep(0.5)
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to read frame from ESP32-CAM stream.")
            self.cap.open(f"http://{esp32_hostname_or_ip}:81/stream")
            self.get_logger().info("Attempting to reopen the stream...")
            time.sleep(0.5)
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.frame_id = "camera_link_optical"
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error converting/publishing frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    stream_getter = StreamGetter()
    try:
        rclpy.spin(stream_getter)
    except KeyboardInterrupt:
        pass
    finally:
        if stream_getter.cap.isOpened():
            stream_getter.cap.release()
        stream_getter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
