#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StringPublisher(Node):
    def __init__(self):
        super().__init__('string_publisher')

        # Create a publisher on 'input_topic'
        self.publisher_ = self.create_publisher(String, 'input_topic', 10)

        # Publish a message every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        # Create and publish a message
        msg = String()
        msg.data = 'Hello from ROS 2!'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StringPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
