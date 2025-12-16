---
id: isaac-sim-setup
title: "Lesson 1: Isaac Sim Setup"
sidebar_position: 1
---

# Lesson 1: Isaac Sim Setup

## Prerequisites

- **RTX GPU required** (RTX 2060 or higher)
- Ubuntu 22.04 with ROS 2 Humble
- Modules 1 & 2 completed

## Install NVIDIA Omniverse Launcher

1. Download from https://www.nvidia.com/en-us/omniverse/download/
2. Extract and run:
```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

3. Sign in with NVIDIA account

## Install Isaac Sim

**In Omniverse Launcher:**
- Navigate to "Exchange" tab
- Search "Isaac Sim"
- Install version **2023.1.1** or later
- Wait for download (~8 GB)

**Verify installation:**
```bash
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

Should launch Isaac Sim GUI.

## Install Isaac ROS

Isaac ROS provides GPU-accelerated perception packages.

```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-visual-slam \
                 ros-humble-isaac-ros-nvblox \
                 ros-humble-isaac-ros-common

# Source ROS
source /opt/ros/humble/setup.bash
```

## Verify ROS 2 Bridge

Isaac Sim includes built-in ROS 2 bridge:

1. Launch Isaac Sim
2. Enable ROS 2 Bridge extension:
   - Window → Extensions
   - Search "ROS2"
   - Enable "omni.isaac.ros2_bridge"

3. Test connection:
```bash
# Terminal 1: Start Isaac Sim (with a scene loaded)
# Terminal 2: Check topics
ros2 topic list
```

You should see Isaac Sim topics like `/clock`, `/joint_states`.

## Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | RTX 2060 | RTX 3080+ |
| VRAM | 6 GB | 12 GB+ |
| RAM | 16 GB | 32 GB |
| Storage | 50 GB | 100 GB SSD |

## Next Steps

- Lesson 2: Generate synthetic training data
- Lesson 3: VSLAM for 3D mapping
- Lesson 4: Autonomous navigation with Nav2
