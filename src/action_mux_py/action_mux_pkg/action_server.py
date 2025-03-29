#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

# Import the custom action interface from the interface package
from my_action_interfaces.action import DoWork


class DoWorkActionServer(Node):
    def __init__(self):
        super().__init__('do_work_action_server')

        # Create the Action Server for the 'DoWork' action
        self._action_server = ActionServer(
            self,
            DoWork,
            'do_work',  # Name of the action
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        # Automatically accept all goals
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Allow clients to cancel goals
        self.get_logger().info('Received goal cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = DoWork.Feedback()

        # Simulate work for 5 seconds
        for i in range(5):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal was canceled')
                return DoWork.Result(success=False)

            # Simulate work by sleeping
            time.sleep(1)

            # Publish feedback (progress percentage)
            feedback_msg.progress = float(i + 1) / 5.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progress: {feedback_msg.progress * 100:.0f}%')

        # If the loop completes, the goal succeeded
        goal_handle.succeed()
        self.get_logger().info('Goal completed successfully')
        return DoWork.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = DoWorkActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
