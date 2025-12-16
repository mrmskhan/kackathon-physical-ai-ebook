#!/usr/bin/env python3
"""
Action Orchestrator

Executes action sequences from LLM planner by calling ROS 2 actions.

Usage:
  ros2 run vla_demos action_orchestrator
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json
import time


class ActionOrchestrator(Node):
    def __init__(self):
        super().__init__('action_orchestrator')

        # Subscribers
        self.action_sub = self.create_subscription(
            String, '/action_sequence', self.action_callback, 10)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Location database
        self.locations = {
            "kitchen": {"x": 5.0, "y": 3.0, "z": 0.0},
            "bedroom": {"x": -2.0, "y": 4.0, "z": 0.0},
            "living_room": {"x": 0.0, "y": 0.0, "z": 0.0},
            "table": {"x": 2.0, "y": 1.0, "z": 0.0},
            "bin": {"x": -3.0, "y": -2.0, "z": 0.0},
            "user": {"x": 0.0, "y": -1.0, "z": 0.0}
        }

        self.get_logger().info("Action Orchestrator ready")

    def action_callback(self, msg):
        """Execute action sequence."""
        try:
            plan = json.loads(msg.data)
            actions = plan.get("actions", [])

            self.get_logger().info(f"Executing {len(actions)} actions")

            for i, action in enumerate(actions):
                self.publish_status(f"Executing action {i+1}/{len(actions)}: {action}")
                success = self.execute_action(action)

                if not success:
                    self.get_logger().error(f"Action failed: {action}")
                    self.publish_status(f"Failed at action {i+1}: {action}")
                    return

            self.publish_status("All actions completed successfully")

        except Exception as e:
            self.get_logger().error(f"Orchestration error: {e}")
            self.publish_status(f"Error: {e}")

    def execute_action(self, action_str):
        """Execute a single action."""
        # Parse action
        if "(" not in action_str:
            self.get_logger().error(f"Invalid action format: {action_str}")
            return False

        action_name, args = action_str.split("(", 1)
        args = args.rstrip(")").strip()

        # Dispatch to handler
        if action_name == "navigate":
            return self.execute_navigate(args)
        elif action_name == "pick":
            return self.execute_pick(args)
        elif action_name == "place":
            return self.execute_place(args)
        elif action_name == "wait":
            return self.execute_wait(int(args))
        elif action_name == "speak":
            return self.execute_speak(args)
        else:
            self.get_logger().error(f"Unknown action: {action_name}")
            return False

    def execute_navigate(self, location):
        """Navigate to location using Nav2."""
        coords = self.locations.get(location.lower())
        if not coords:
            self.get_logger().error(f"Unknown location: {location}")
            return False

        self.get_logger().info(f"Navigating to {location} at {coords}")

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords["x"]
        goal_msg.pose.pose.position.y = coords["y"]
        goal_msg.pose.pose.position.z = coords["z"]
        goal_msg.pose.pose.orientation.w = 1.0

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            return False

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Arrived at {location}")
            return True
        else:
            self.get_logger().error(f"Navigation failed with status {result.status}")
            return False

    def execute_pick(self, object_name):
        """Pick up object (simulated)."""
        self.get_logger().info(f"Picking up {object_name}")
        time.sleep(2)  # Simulate gripper action
        self.get_logger().info(f"Picked up {object_name}")
        return True

    def execute_place(self, location):
        """Place object (simulated)."""
        self.get_logger().info(f"Placing object at {location}")
        time.sleep(2)  # Simulate release
        self.get_logger().info(f"Placed object")
        return True

    def execute_wait(self, seconds):
        """Wait for N seconds."""
        self.get_logger().info(f"Waiting {seconds} seconds")
        time.sleep(seconds)
        return True

    def execute_speak(self, text):
        """Speak text (simulated)."""
        self.get_logger().info(f"Speaking: {text}")
        return True

    def publish_status(self, status):
        """Publish robot status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionOrchestrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
