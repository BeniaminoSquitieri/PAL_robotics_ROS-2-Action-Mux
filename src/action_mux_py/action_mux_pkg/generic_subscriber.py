#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import importlib


class GenericSubscriber(Node):
    def __init__(self):
        super().__init__('generic_subscriber')

        # Declare parameters to specify topic name and message type
        self.declare_parameter('topic_name', 'input_topic')
        self.declare_parameter('msg_type', 'std_msgs.msg.String')

        # Retrieve the parameters
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        msg_type_str = self.get_parameter('msg_type').get_parameter_value().string_value

        # Dynamically import the message type
        module_name, type_name = msg_type_str.rsplit('.', 1)
        module = importlib.import_module(module_name)
        msg_type = getattr(module, type_name)

        # Create the subscriber
        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.callback,
            10
        )

        self.get_logger().info(f'Subscribed to {topic_name} with type {msg_type_str}')

    def callback(self, msg):
        # Print the message and its type
        self.get_logger().info(f'Received message: {str(msg)}')
        self.get_logger().info(f'Message type: {str(type(msg))}')


def main(args=None):
    rclpy.init(args=args)
    node = GenericSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
