---
id: implementation
title: "Implementation Guide"
sidebar_position: 3
---

# Implementation Guide

Step-by-step instructions to build the complete VLA system.

## Phase 1: Environment Setup

**1. Create Workspace**
```bash
mkdir -p ~/vla_capstone/src
cd ~/vla_capstone
```

**2. Clone Dependencies**
```bash
# From Module 2
git clone <your-gazebo-humanoid-repo> src/gazebo_humanoid

# From Module 3
git clone <isaac-ros-vslam> src/isaac_ros_visual_slam

# From Module 4
mkdir -p src/vla_demos/vla_demos
# Copy all Module 4 Python scripts to src/vla_demos/vla_demos/
```

**3. Install Dependencies**
```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-localization python3-pip portaudio19-dev

pip3 install openai whisper pyaudio
```

**4. Set API Keys**
```bash
echo "export OPENAI_API_KEY='sk-...'" >> ~/.bashrc
source ~/.bashrc
```

## Phase 2: Launch System Components

**1. Start Gazebo Simulation**

Terminal 1:
```bash
source ~/vla_capstone/install/setup.bash
ros2 launch gazebo_humanoid spawn_humanoid.launch.py
```

**2. Launch VSLAM**

Terminal 2:
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**3. Start Nav2**

Terminal 3:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/nav2_params.yaml
```

**4. Launch VLA Pipeline**

Terminal 4:
```bash
# Whisper node
ros2 run vla_demos whisper_ros_node.py
```

Terminal 5:
```bash
# LLM planner
ros2 run vla_demos llm_planner.py
```

Terminal 6:
```bash
# Action orchestrator
ros2 run vla_demos action_orchestrator.py
```

## Phase 3: Quick Integration Test

**Test 1: Voice Recognition**
```bash
# Speak into microphone: "Robot, go to the kitchen"
# Check terminal output:
ros2 topic echo /voice_command
# Expected: data: "go to the kitchen"
```

**Test 2: LLM Planning**
```bash
# Publish manual command
ros2 topic pub /voice_command std_msgs/String "data: 'go to the kitchen'"

# Check action sequence
ros2 topic echo /action_sequence
# Expected: {"actions": ["navigate(kitchen)"], ...}
```

**Test 3: Navigation**
```bash
# Should see robot moving in Gazebo
# Check Nav2 feedback
ros2 topic echo /navigate_to_pose/_action/feedback
```

## Phase 4: Full Launch File

**Create Single Launch File** (`capstone.launch.py`):

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription('gazebo_humanoid/spawn_humanoid.launch.py'),

        # VSLAM
        IncludeLaunchDescription('isaac_ros_visual_slam.launch.py'),

        # Nav2
        IncludeLaunchDescription('nav2_bringup/navigation_launch.py',
            launch_arguments={'use_sim_time': 'true'}.items()),

        # Whisper
        Node(package='vla_demos', executable='whisper_ros_node.py'),

        # LLM Planner
        Node(package='vla_demos', executable='llm_planner.py'),

        # Action Orchestrator
        Node(package='vla_demos', executable='action_orchestrator.py'),

        # RViz2
        Node(package='rviz2', executable='rviz2',
            arguments=['-d', 'capstone.rviz'])
    ])
```

**Launch Everything:**
```bash
ros2 launch vla_demos capstone.launch.py
```

## Phase 5: Configuration

**1. Tune Nav2 Parameters** (`nav2_params.yaml`):

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      min_vel_x: -0.5
      max_vel_theta: 1.0
      acc_lim_x: 0.3
      acc_lim_theta: 2.0

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.125
      downsample_costmap: false
```

**2. Update Location Database** (`action_orchestrator.py`):

```python
self.locations = {
    "kitchen": {"x": 5.0, "y": 3.0, "z": 0.0},
    "bedroom": {"x": -2.0, "y": 4.0, "z": 0.0},
    "living_room": {"x": 0.0, "y": 0.0, "z": 0.0},
    # Add your environment's locations
}
```

**3. Customize LLM System Prompt** (`llm_planner.py`):

```python
system_prompt = """You are a robot task planner.
Convert natural language into JSON action sequences.

Available actions:
- navigate(location): kitchen, bedroom, living_room
- pick(object): cup, book, trash
- place(location)
- speak(text)

Output JSON only, no explanations."""
```

## Phase 6: Testing Workflow

**Test Scenarios:**

1. **Simple Navigation**
   - Command: "Go to the kitchen"
   - Expected: Robot navigates to kitchen coordinates

2. **Multi-Step Task**
   - Command: "Go to the bedroom and come back"
   - Expected: Navigate to bedroom, then return to origin

3. **Object Retrieval** (requires manipulation)
   - Command: "Bring me the cup from the kitchen"
   - Expected: Navigate → Pick → Return → Place

4. **Error Recovery**
   - Command: "Go to the moon"
   - Expected: LLM rejects invalid location or asks for clarification

## Phase 7: Performance Monitoring

**Monitor Latency:**
```bash
# Check message timestamps
ros2 topic echo /voice_command --field data --once | ts
ros2 topic echo /action_sequence --once | ts
```

**Check CPU/GPU Usage:**
```bash
htop  # CPU usage
nvidia-smi -l 1  # GPU usage
```

**View ROS 2 Graph:**
```bash
rqt_graph
```

## Phase 8: Hardware Deployment (Optional)

**1. Transfer to Jetson**
```bash
# On workstation
tar -czf vla_capstone.tar.gz ~/vla_capstone
scp vla_capstone.tar.gz username@jetson-orin:~/

# On Jetson
tar -xzf vla_capstone.tar.gz
cd vla_capstone
colcon build
```

**2. Update Launch File**
```python
# Change use_sim_time to False
launch_arguments={'use_sim_time': 'false'}.items()
```

**3. Connect Real Sensors**
```bash
# RealSense
rs-enumerate-devices  # Verify detected

# Microphone
arecord -l  # List devices
```

**4. Test on Hardware**
```bash
ros2 launch vla_demos capstone.launch.py
```

## Next Steps

See Troubleshooting for common issues and solutions.
