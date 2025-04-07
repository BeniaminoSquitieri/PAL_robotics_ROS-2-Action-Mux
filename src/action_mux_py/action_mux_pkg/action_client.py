#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

# Import the custom action interface
from my_action_interfaces.action import DoWork


class ActionMuxClient(Node):
    def __init__(self):
        super().__init__('action_mux_client')

        # Action client for 'DoWork'
        self._action_client = ActionClient(self, DoWork, 'do_work')
        self.current_goal_handle = None  # Keep track of the current goal

        # Subscribe to a topic to receive commands (e.g., "input_topic")
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.topic_callback,
            10
        )
        self.get_logger().info('Subscribed to "input_topic"')

    def topic_callback(self, msg):
        self.get_logger().info(f'Received message: "{msg.data}"')
        self.get_logger().info(f'Message type: {str(type(msg))}')

        # Cancel any current goal before sending a new one
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling current goal')
            self.current_goal_handle.cancel_goal()

        # Create and send a new goal
        goal_msg = DoWork.Goal()
        goal_msg.work_duration = 5  # Set the duration to 5 seconds
        self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        self.get_logger().info('Sending new goal...')
        self._action_client.wait_for_server()

        # Send the goal asynchronously
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')
        self.current_goal_handle = goal_handle

        # Wait for the result asynchronously
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: success={result.success}')
        self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    client = ActionMuxClient()
    rclpy.spin(client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
