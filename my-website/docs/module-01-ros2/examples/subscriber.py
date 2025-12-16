#!/usr/bin/env python3
"""
ROS 2 Subscriber Example - Minimal Subscriber Node

This example demonstrates the publish-subscribe pattern in ROS 2.
The subscriber listens to messages on the /chatter topic and logs them.

Usage:
    python3 subscriber.py

Topics:
    /chatter (std_msgs/String): Subscribed topic

Author: Physical AI & Humanoid Robotics E-Book
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber node that listens to string messages on the /chatter topic.

    This node demonstrates:
    - Creating a subscription
    - Using callbacks to handle incoming messages
    - Logging received data
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription: message type, topic name, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        # Prevent unused variable warning (Python idiom)
        self.subscription

        self.get_logger().info('Minimal Subscriber has started')

    def listener_callback(self, msg):
        """
        Callback function triggered when a message is received on /chatter.

        Args:
            msg (std_msgs/String): The received message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize and run the subscriber node.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = MinimalSubscriber()

    try:
        # Keep the node running (spin blocks until shutdown)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
