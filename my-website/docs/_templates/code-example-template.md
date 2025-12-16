# Code Example Template

Use this template for standalone code examples referenced from lessons.

## Example: [Descriptive Name]

**Purpose**: [Brief 1-sentence description of what this example demonstrates]

**Difficulty**: [Beginner/Intermediate/Advanced]

**Estimated Time**: [X minutes]

## Prerequisites

**System Requirements**:
- Ubuntu 22.04 LTS
- ROS 2 Humble installed
- [Additional requirements]

**Knowledge**:
- Understanding of [concept 1]
- Familiarity with [concept 2]

**Installation**:
```bash
# Install required packages
sudo apt update
sudo apt install [package-list]
```

## Setup

**Step 1: Environment Setup**
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src/examples
cd ~/ros2_ws/src/examples
```

**Step 2: Create File**
```bash
touch example-name.py
chmod +x example-name.py
```

## Code

```python
#!/usr/bin/env python3
"""
Brief description of what this code does.

Example usage:
    python3 example-name.py
"""

import rclpy
from rclpy.node import Node
# Additional imports

class ExampleNode(Node):
    """
    Brief class description.
    """

    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('Example node started')

        # Initialize components
        # ...

    def example_method(self):
        """Method description."""
        # Implementation
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Execution

**Run the example**:
```bash
python3 example-name.py
```

**Expected Output**:
```
[INFO] [timestamp] [example_node]: Example node started
[INFO] [timestamp] [example_node]: [Example output...]
```

## Explanation

### Key Components

**1. Node Initialization**
```python
super().__init__('example_node')
```
[Explanation of why this is important]

**2. Core Logic**
```python
# Highlighted code section
```
[Explanation of how this works]

**3. Cleanup**
```python
node.destroy_node()
rclpy.shutdown()
```
[Explanation of proper shutdown]

## Testing

**Verify the output**:
```bash
# In another terminal
ros2 topic list
# Should show: [expected topics]

ros2 node list
# Should show: /example_node
```

## Troubleshooting

### Error: "ModuleNotFoundError: No module named 'rclpy'"

**Solution**:
```bash
# Ensure ROS 2 environment is sourced
source /opt/ros/humble/setup.bash
```

### Error: "Permission denied"

**Solution**:
```bash
# Make file executable
chmod +x example-name.py
```

## Extensions

Try these modifications to extend your learning:

1. **Modification 1**: [Description and expected behavior change]
2. **Modification 2**: [Description and expected behavior change]
3. **Challenge**: [More advanced modification]

## Related Examples

- [Related Example 1](./related-example-1.md)
- [Related Example 2](./related-example-2.md)

## References

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [Relevant Tutorial](https://link)
