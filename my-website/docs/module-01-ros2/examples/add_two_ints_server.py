#!/usr/bin/env python3
"""
ROS 2 Service Server Example - Add Two Ints

This example demonstrates the service pattern in ROS 2.
The server provides a service to add two integers and return the sum.

Usage:
    python3 add_two_ints_server.py

Services:
    /add_two_ints (example_interfaces/AddTwoInts): Adds two integers

Author: Physical AI & Humanoid Robotics E-Book
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    A service server that provides the /add_two_ints service.

    This node demonstrates:
    - Creating a service server
    - Processing service requests
    - Sending service responses
    """

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service: service type, service name, callback function
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Add Two Ints Server Ready')

    def add_two_ints_callback(self, request, response):
        """
        Service callback function that handles add_two_ints requests.

        Args:
            request (AddTwoInts.Request): Contains a (int64) and b (int64)
            response (AddTwoInts.Response): Contains sum (int64)

        Returns:
            AddTwoInts.Response: The populated response object
        """
        # Perform the addition
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(
            f'Incoming request: a={request.a}, b={request.b}'
        )
        self.get_logger().info(f'Sending response: sum={response.sum}')

        return response


def main(args=None):
    """
    Main function to initialize and run the service server.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = AddTwoIntsServer()

    try:
        # Keep the node running to serve requests
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Server stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
