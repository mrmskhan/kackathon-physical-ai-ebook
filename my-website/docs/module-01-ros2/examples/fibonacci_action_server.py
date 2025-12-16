#!/usr/bin/env python3
"""
ROS 2 Action Server Example - Fibonacci Sequence

This example demonstrates the action pattern in ROS 2.
The server computes a Fibonacci sequence and provides feedback during execution.

Usage:
    python3 fibonacci_action_server.py

Actions:
    /fibonacci (action_tutorials_interfaces/Fibonacci): Computes Fibonacci sequence

Author: Physical AI & Humanoid Robotics E-Book
License: Apache 2.0
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """
    An action server that computes Fibonacci sequences.

    This node demonstrates:
    - Creating an action server
    - Publishing feedback during execution
    - Returning final results
    - Handling goal execution
    """

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server: node, action type, action name, callback
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci Action Server Started')

    def execute_callback(self, goal_handle):
        """
        Action execution callback function.

        This function is called when a goal is accepted and executes the
        Fibonacci sequence computation with periodic feedback.

        Args:
            goal_handle: Handle to the goal being executed

        Returns:
            Fibonacci.Result: Final Fibonacci sequence
        """
        self.get_logger().info('Executing goal...')

        # Initialize feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Generate Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Publish feedback
            self.get_logger().info(
                f'Feedback: {feedback_msg.partial_sequence}'
            )
            goal_handle.publish_feedback(feedback_msg)

            # Compute next Fibonacci number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] +
                feedback_msg.partial_sequence[i - 1]
            )

            # Simulate work (0.5 seconds per iteration)
            time.sleep(0.5)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Prepare final result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.get_logger().info(f'Goal succeeded! Result: {result.sequence}')

        return result


def main(args=None):
    """
    Main function to initialize and run the action server.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = FibonacciActionServer()

    try:
        # Keep the node running to accept goals
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Action server stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
