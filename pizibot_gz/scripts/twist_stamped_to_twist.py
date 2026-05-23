#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStampedToTwist(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')
        self.pub = self.create_publisher(Twist, 'cmd_vel_gz', 10)
        self.create_subscription(TwistStamped, 'cmd_vel_out', self._cb, 10)

    def _cb(self, msg: TwistStamped):
        self.pub.publish(msg.twist)


def main():
    rclpy.init()
    rclpy.spin(TwistStampedToTwist())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
