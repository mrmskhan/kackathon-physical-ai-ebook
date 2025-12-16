#!/usr/bin/env python3
"""
ROS 2 Service Client Example - Add Two Ints

This example demonstrates calling a ROS 2 service from a client.
The client sends a request to add 3 + 5 and expects the result 8.

Usage:
    python3 add_two_ints_client.py

Services:
    /add_two_ints (example_interfaces/AddTwoInts): Service to call

Author: Physical AI & Humanoid Robotics E-Book
License: Apache 2.0
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """
    A service client that calls the /add_two_ints service.

    This node demonstrates:
    - Creating a service client
    - Waiting for service availability
    - Sending asynchronous service requests
    - Handling service responses
    """

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client: service type, service name
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Create request object
        self.request = AddTwoInts.Request()

        self.get_logger().info('Add Two Ints Client Ready')

    def send_request(self, a, b):
        """
        Send a service request to add two integers.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            Future: Asynchronous future object for the response
        """
        self.request.a = a
        self.request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Call service asynchronously
        self.future = self.client.call_async(self.request)

        return self.future


def main(args=None):
    """
    Main function to initialize and run the service client.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = AddTwoIntsClient()

    # Send request (you can modify these values)
    future = node.send_request(3, 5)

    # Spin until the future completes
    rclpy.spin_until_future_complete(node, future)

    # Check if the call succeeded
    if future.result() is not None:
        node.get_logger().info(f'Result: {future.result().sum}')
    else:
        node.get_logger().error(
            f'Service call failed: {future.exception()}'
        )

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
