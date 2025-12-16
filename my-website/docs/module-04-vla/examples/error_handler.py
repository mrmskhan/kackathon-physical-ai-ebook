#!/usr/bin/env python3
"""
Error Handler Module

Handles errors from navigation, manipulation, and command parsing.

Usage:
  from error_handler import ErrorHandler
  handler = ErrorHandler()
  recovery = handler.handle_navigation_error("GOAL_BLOCKED")
"""


class ErrorHandler:
    """Handle errors and suggest recovery strategies."""

    def __init__(self):
        self.error_counts = {}
        self.max_retries = 3

    def handle_navigation_error(self, error_code):
        """Handle navigation errors."""
        strategies = {
            "GOAL_BLOCKED": "retry_with_recovery",
            "GOAL_UNREACHABLE": "request_alternative",
            "PATH_BLOCKED": "replan_path",
            "TIMEOUT": "increase_timeout",
            "UNKNOWN": "manual_intervention"
        }

        strategy = strategies.get(error_code, "manual_intervention")

        # Track retry count
        self.error_counts[error_code] = self.error_counts.get(error_code, 0) + 1

        if self.error_counts[error_code] > self.max_retries:
            return "manual_intervention"

        return strategy

    def handle_manipulation_error(self, error_code):
        """Handle manipulation errors."""
        strategies = {
            "OBJECT_NOT_FOUND": "request_user_help",
            "GRASP_FAILED": "retry_different_angle",
            "OBJECT_DROPPED": "re_grasp",
            "GRIPPER_ERROR": "reset_gripper"
        }

        return strategies.get(error_code, "manual_intervention")

    def handle_command_error(self, error_type):
        """Handle command parsing errors."""
        responses = {
            "UNCLEAR": "Could you please repeat that?",
            "INVALID_LOCATION": "I don't know that location. Try: kitchen, bedroom, living room.",
            "INVALID_OBJECT": "I don't recognize that object. Try: cup, book, trash.",
            "AMBIGUOUS": "That command is unclear. Please be more specific.",
            "UNSAFE": "That action might be unsafe. Please confirm."
        }

        return responses.get(error_type, "I didn't understand that command.")

    def reset_error_count(self, error_code):
        """Reset error count after successful recovery."""
        if error_code in self.error_counts:
            del self.error_counts[error_code]

    def get_error_stats(self):
        """Get error statistics."""
        return {
            "total_errors": sum(self.error_counts.values()),
            "error_breakdown": self.error_counts
        }


# Recovery strategies implementation

def retry_with_recovery(nav_client, goal):
    """Retry navigation with recovery behaviors enabled."""
    # Enable recovery behaviors in Nav2
    goal.behavior_tree = "navigate_w_replanning_and_recovery"
    return nav_client.send_goal_async(goal)


def request_alternative(llm_planner, failed_location):
    """Ask LLM for alternative location."""
    prompt = f"The robot cannot reach {failed_location}. Suggest an alternative location."
    return llm_planner.generate_alternative(prompt)


def replan_path(nav_client, goal):
    """Force path replanning."""
    # Cancel current goal and replan
    nav_client.cancel_goal()
    return nav_client.send_goal_async(goal)


def request_user_help(status_pub, object_name):
    """Request user to help locate object."""
    message = f"I cannot find {object_name}. Can you help me locate it?"
    status_pub.publish(message)


def retry_different_angle(manipulation_client, object_name):
    """Retry grasp from different angle."""
    angles = [0, 45, 90, 135, 180]
    for angle in angles:
        result = manipulation_client.grasp(object_name, approach_angle=angle)
        if result.success:
            return True
    return False


# Example usage
if __name__ == "__main__":
    handler = ErrorHandler()

    # Test navigation error
    error = "GOAL_BLOCKED"
    strategy = handler.handle_navigation_error(error)
    print(f"Navigation error '{error}' → Strategy: {strategy}")

    # Test manipulation error
    error = "GRASP_FAILED"
    strategy = handler.handle_manipulation_error(error)
    print(f"Manipulation error '{error}' → Strategy: {strategy}")

    # Test command error
    error = "UNCLEAR"
    response = handler.handle_command_error(error)
    print(f"Command error '{error}' → Response: {response}")

    # Get stats
    stats = handler.get_error_stats()
    print(f"Error stats: {stats}")
