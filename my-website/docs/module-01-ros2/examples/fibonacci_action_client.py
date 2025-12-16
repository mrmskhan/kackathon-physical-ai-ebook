#!/usr/bin/env python3
"""
ROS 2 Action Client Example - Fibonacci Sequence

This example demonstrates calling a ROS 2 action from a client.
The client sends a goal to compute a Fibonacci sequence and receives feedback.

Usage:
    python3 fibonacci_action_client.py

Actions:
    /fibonacci (action_tutorials_interfaces/Fibonacci): Action to call

Author: Physical AI & Humanoid Robotics E-Book
License: Apache 2.0
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """
    An action client that requests Fibonacci sequence computation.

    This node demonstrates:
    - Creating an action client
    - Sending goals asynchronously
    - Receiving feedback during execution
    - Handling goal acceptance and results
    """

    def __init__(self):
        super().__init__('fibonacci_action_client')

        # Create action client: node, action type, action name
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

        self.get_logger().info('Fibonacci Action Client Ready')

    def send_goal(self, order):
        """
        Send a goal to compute a Fibonacci sequence.

        Args:
            order (int): Number of Fibonacci numbers to compute
        """
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: Fibonacci order {order}')

        # Send goal asynchronously with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Register callback for when goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback when the server accepts or rejects the goal.

        Args:
            future: Future object containing the goal handle
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server, awaiting result...')

        # Get result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback when the final result is available.

        Args:
            future: Future object containing the result
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

        # Shutdown after receiving result
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Callback for feedback messages during goal execution.

        Args:
            feedback_msg: Feedback message from the action server
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')


def main(args=None):
    """
    Main function to initialize and run the action client.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = FibonacciActionClient()

    # Send goal (compute Fibonacci sequence with order 10)
    node.send_goal(10)

    try:
        # Keep the node running to receive feedback and results
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Action client stopped by user')
    finally:
        # Cleanup
        node.destroy_node()


if __name__ == '__main__':
    main()
