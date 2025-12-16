#!/usr/bin/env python3
"""
LLM Task Planner Node

Subscribes to /voice_command, generates action sequences using OpenAI API,
publishes to /action_sequence.

Usage:
  export OPENAI_API_KEY="sk-..."
  ros2 run vla_demos llm_planner
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import json
import os


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10)

        # Publishers
        self.action_pub = self.create_publisher(String, '/action_sequence', 10)

        # OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY not set!")
            raise ValueError("Missing OpenAI API key")

        self.client = OpenAI(api_key=api_key)

        # System prompt
        self.system_prompt = """You are a robot task planner. Convert natural language commands into JSON action sequences.

Available actions:
- navigate(location) - Move to location (kitchen, bedroom, living_room, table, bin)
- pick(object) - Grasp object (cup, book, trash, phone)
- place(location) - Release object at location
- wait(seconds) - Pause for N seconds
- speak(text) - Say something to user

Output format (JSON only, no explanation):
{"actions": ["action1(arg)", "action2(arg)"], "estimated_time": 60}

Examples:
Input: "go to the kitchen"
Output: {"actions": ["navigate(kitchen)"], "estimated_time": 15}

Input: "bring me the cup from the table"
Output: {"actions": ["navigate(table)", "pick(cup)", "navigate(user)", "place(user)"], "estimated_time": 45}
"""

        self.get_logger().info("LLM Planner ready")

    def command_callback(self, msg):
        """Process voice command with LLM."""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        try:
            # Generate plan with OpenAI
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0.0,  # Deterministic output
                max_tokens=200
            )

            plan_text = response.choices[0].message.content.strip()
            self.get_logger().info(f"LLM response: {plan_text}")

            # Parse JSON
            plan = json.loads(plan_text)

            # Validate actions
            if not self.validate_plan(plan):
                self.get_logger().error("Invalid action plan")
                return

            # Publish action sequence
            action_msg = String()
            action_msg.data = json.dumps(plan)
            self.action_pub.publish(action_msg)

            self.get_logger().info(f"Published action sequence: {plan['actions']}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse LLM output: {e}")
        except Exception as e:
            self.get_logger().error(f"LLM error: {e}")

    def validate_plan(self, plan):
        """Validate action plan structure."""
        if "actions" not in plan:
            return False

        valid_actions = ["navigate", "pick", "place", "wait", "speak"]

        for action in plan["actions"]:
            action_name = action.split("(")[0]
            if action_name not in valid_actions:
                self.get_logger().warn(f"Unknown action: {action_name}")
                return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
