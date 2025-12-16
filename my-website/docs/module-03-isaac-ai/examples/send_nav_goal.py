#!/usr/bin/env python3
"""
Send Navigation Goals to Nav2

Sends a sequence of waypoints to Nav2 using ROS 2 actions.

Usage:
  # Single goal
  python send_nav_goal.py --x 5.0 --y 3.0 --yaw 1.57

  # Waypoint patrol
  python send_nav_goal.py --patrol
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.navigator = BasicNavigator()

    def send_goal(self, x, y, yaw=0.0):
        """Send a single navigation goal."""

        self.get_logger().info(f"Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = float(yaw / 2.0)
        goal_pose.pose.orientation.w = float((1 - (yaw / 2.0) ** 2) ** 0.5)

        self.navigator.goToPose(goal_pose)

        # Wait for navigation to complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"Distance remaining: {feedback.distance_remaining:.2f}m"
                )
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal reached!")
            return True
        else:
            self.get_logger().error(f"Navigation failed: {result}")
            return False

    def patrol_waypoints(self):
        """Patrol a sequence of waypoints."""

        waypoints = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14),
            (0.0, 0.0, -1.57)
        ]

        self.get_logger().info(f"Starting patrol with {len(waypoints)} waypoints")

        for i, (x, y, yaw) in enumerate(waypoints):
            self.get_logger().info(f"Waypoint {i+1}/{len(waypoints)}")
            success = self.send_goal(x, y, yaw)
            if not success:
                self.get_logger().error(f"Failed at waypoint {i+1}, aborting patrol")
                return False
            time.sleep(2.0)  # Pause between waypoints

        self.get_logger().info("Patrol complete!")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=0.0, help='Goal X position')
    parser.add_argument('--y', type=float, default=0.0, help='Goal Y position')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw (radians)')
    parser.add_argument('--patrol', action='store_true', help='Run waypoint patrol')
    args = parser.parse_args()

    rclpy.init()
    node = NavGoalSender()

    try:
        if args.patrol:
            node.patrol_waypoints()
        else:
            node.send_goal(args.x, args.y, args.yaw)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
