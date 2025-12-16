---
id: action-integration
title: "Lesson 3: Action Integration with ROS 2"
sidebar_position: 3
---

# Lesson 3: Action Integration with ROS 2

## Map LLM → ROS 2 Actions

Convert LLM action sequences into ROS 2 action calls.

**LLM output:**
```json
{"actions": ["navigate(kitchen)", "pick(cup)"]}
```

**ROS 2 execution:**
1. Call Nav2 action: `NavigateToPose` with kitchen coordinates
2. Call manipulation action: `PickObject` with cup ID

## ROS 2 Actions Review

Actions = long-running tasks with feedback.

**Action components:**
- **Goal**: Target (e.g., destination pose)
- **Feedback**: Progress updates
- **Result**: Success/failure

**Example: Nav2 navigation**
```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"
```

## Action Orchestrator

See `examples/action_orchestrator.py` - executes action sequences from LLM.

**Key functions:**
- `execute_navigate(location)` - Send Nav2 goal
- `execute_pick(object)` - Call gripper action
- `execute_place(location)` - Release object
- `execute_wait(seconds)` - Pause execution

## Location Database

Map natural language → coordinates:

```python
LOCATIONS = {
    "kitchen": {"x": 5.0, "y": 3.0},
    "bedroom": {"x": -2.0, "y": 4.0},
    "living_room": {"x": 0.0, "y": 0.0}
}

def get_coordinates(location_name):
    return LOCATIONS.get(location_name.lower())
```

## Action Execution Flow

```python
def execute_action_sequence(actions):
    for action in actions:
        action_type, arg = parse_action(action)

        if action_type == "navigate":
            coords = get_coordinates(arg)
            send_nav2_goal(coords)
            wait_for_completion()

        elif action_type == "pick":
            send_pick_action(arg)
            wait_for_completion()
```

See full implementation in `examples/action_orchestrator.py`.

## Error Handling

**Navigation failures:**
- Obstacle blocking path → Retry with different planner
- Goal unreachable → Request alternative location from LLM

**Manipulation failures:**
- Object not found → Request user help
- Grasp failed → Retry with different approach angle

```python
def handle_navigation_error(error_code):
    if error_code == "GOAL_BLOCKED":
        return "retry_with_recovery"
    elif error_code == "GOAL_UNREACHABLE":
        return "request_alternative"
```

See `examples/error_handler.py` for full error handling logic.

## Feedback to User

Report progress via:
- Text-to-speech: "Navigating to kitchen..."
- LED indicators: Green = success, Red = error
- ROS 2 topic: `/robot_status`

```python
from std_msgs.msg import String

status_pub = node.create_publisher(String, '/robot_status', 10)
status_pub.publish(String(data="Picking up cup"))
```

## Parallel Actions

Some actions can run concurrently:

```python
# Sequential (default)
navigate(kitchen)  # Wait for completion
pick(cup)          # Then execute

# Parallel (advanced)
async def navigate_and_speak():
    await asyncio.gather(
        navigate(kitchen),
        speak("On my way!")
    )
```

## Testing

**Unit test actions:**
```bash
ros2 run vla_demos test_navigate --location kitchen
ros2 run vla_demos test_pick --object cup
```

**Integration test:**
```bash
ros2 run vla_demos action_orchestrator \
  --actions "navigate(kitchen),pick(cup)"
```

## Next Lesson

Combine everything: voice → Whisper → LLM → actions in end-to-end capstone demo.
