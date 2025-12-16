#!/usr/bin/env python3
"""
ROS 2 Publisher Example - Minimal Publisher Node

This example demonstrates the publish-subscribe pattern in ROS 2.
The publisher sends "Hello World" messages to the /chatter topic every second.

Usage:
    python3 publisher.py

Topics:
    /chatter (std_msgs/String): Published messages

Author: Physical AI & Humanoid Robotics E-Book
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher node that sends string messages to the /chatter topic.

    This node demonstrates:
    - Creating a publisher
    - Using a timer for periodic publishing
    - Logging messages
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer: period (seconds), callback function
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Message counter
        self.i = 0

        self.get_logger().info('Minimal Publisher has started')

    def timer_callback(self):
        """
        Timer callback function called every 1.0 seconds.
        Creates and publishes a String message to /chatter topic.
        """
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the publisher node.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = MinimalPublisher()

    try:
        # Keep the node running (spin blocks until shutdown)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
