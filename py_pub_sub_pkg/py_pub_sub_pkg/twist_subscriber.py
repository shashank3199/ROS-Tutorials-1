#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('twist_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.twist_callback,
            10)
        self.subscription  # prevent unused variable warning

    def twist_callback(self, msg):
        self.get_logger().info(f'Received twist - linear.x: {msg.twist.linear.x}, angular.z: {msg.twist.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = TwistSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()