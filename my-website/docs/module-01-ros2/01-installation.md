---
id: installation
title: "Lesson 1: ROS 2 Installation"
sidebar_position: 2
---

# Lesson 1: ROS 2 Installation

Learn how to install ROS 2 Humble on Ubuntu 22.04 and configure your development environment.

## Learning Objectives

- Install ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS
- Configure the ROS 2 environment in your shell
- Verify the installation with basic ROS 2 commands
- Understand the ROS 2 workspace structure

## Prerequisites

import CalloutBox from '@site/src/components/CalloutBox';

<CalloutBox type="prerequisite" title="Before You Begin">

- Ubuntu 22.04 LTS installed (physical machine or VM)
- Stable internet connection for downloading packages (~2GB)
- Sudo privileges on your system

</CalloutBox>

## Why ROS 2 Humble?

**ROS 2 Humble Hawksbill** is a Long-Term Support (LTS) release supported until May 2027. It's the recommended version for production robotics applications and provides:

- **Real-time capable**: Deterministic communication for time-critical tasks
- **Cross-platform**: Linux, macOS, Windows support
- **Production-ready**: Battle-tested in commercial robots (Boston Dynamics, ABB, NASA)
- **Python 3 and C++17**: Modern language features

## Installation Steps

### Step 1: Set Locale

Ensure your system supports UTF-8 encoding:

```bash
locale  # Check current settings
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Why this matters**: ROS 2 uses UTF-8 for message serialization and logging.

### Step 2: Setup Sources

Add the ROS 2 apt repository:

```bash
# Add ROS 2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Humble Desktop

Install the full desktop version (includes RViz2, demos, tutorials):

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
```

**Installation size**: ~1.5GB
**Installation time**: 5-10 minutes (depending on internet speed)

<CalloutBox type="info" title="Desktop vs Base">

- **ros-humble-desktop**: Includes RViz2, demos, tutorials (recommended for learning)
- **ros-humble-ros-base**: Minimal installation without GUI tools (for servers/embedded)

</CalloutBox>

### Step 4: Install Development Tools

Install colcon (ROS 2 build tool) and other development utilities:

```bash
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep python3-argcomplete -y
```

**Colcon** is the build system for ROS 2 packages (similar to catkin in ROS 1).

### Step 5: Environment Setup

Source the ROS 2 setup script to configure environment variables:

```bash
source /opt/ros/humble/setup.bash
```

**Make it permanent** by adding to your shell configuration:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

<CalloutBox type="tip" title="Quick Tip">

If you work with multiple ROS distributions, consider using a ROS environment switcher script instead of hardcoding in `.bashrc`.

</CalloutBox>

### Step 6: Initialize rosdep

`rosdep` resolves package dependencies automatically:

```bash
sudo rosdep init
rosdep update
```

## Verification

Verify your installation with these commands:

### Check ROS 2 Version

```bash
ros2 --version
# Expected output: ros2 cli version: 0.18.x
```

### List Available Commands

```bash
ros2 --help
```

You should see commands like: `run`, `topic`, `service`, `node`, `param`, `launch`, etc.

### Run a Demo Node

Test the installation with a built-in talker demo:

```bash
# Terminal 1: Start talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener (open a new terminal)
ros2 run demo_nodes_cpp listener
```

**Expected behavior**: The talker publishes "Hello World" messages, and the listener echoes them.

<CalloutBox type="warning" title="Troubleshooting">

If nodes can't find each other, check your `ROS_DOMAIN_ID`:

```bash
echo $ROS_DOMAIN_ID
```

If blank, set it explicitly:

```bash
export ROS_DOMAIN_ID=0  # Use any number 0-101
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

</CalloutBox>

## Understanding the ROS 2 Environment

When you source `/opt/ros/humble/setup.bash`, several environment variables are configured:

| Variable | Purpose | Example Value |
|----------|---------|---------------|
| `ROS_VERSION` | ROS major version | `2` |
| `ROS_DISTRO` | Distribution name | `humble` |
| `ROS_PYTHON_VERSION` | Python version | `3` |
| `ROS_DOMAIN_ID` | DDS domain (network isolation) | `0` (default) |

Check all ROS environment variables:

```bash
printenv | grep ROS
```

## Workspace Structure

ROS 2 uses a standardized workspace layout:

```
~/ros2_ws/              # Workspace root
├── src/                # Source code (your packages go here)
├── build/              # Build artifacts (generated by colcon)
├── install/            # Installed packages (executables, libraries)
└── log/                # Build and runtime logs
```

We'll create our first workspace in Lesson 2.

## Common Installation Issues

import TroubleshootingBox from '@site/src/components/TroubleshootingBox';

<TroubleshootingBox
  issue="E: Unable to locate package ros-humble-desktop"
  symptom="Package not found error during apt install"
  cause="ROS 2 repository not properly added or GPG key missing"
  solution="sudo apt update && sudo apt install curl && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
  verification="apt-cache policy ros-humble-desktop"
/>

<TroubleshootingBox
  issue="ros2: command not found"
  symptom="Shell doesn't recognize ros2 commands"
  cause="ROS 2 environment not sourced"
  solution="source /opt/ros/humble/setup.bash"
  verification="ros2 --version"
/>

## Next Steps

Now that ROS 2 is installed, you're ready to learn about **nodes and topics** - the fundamental building blocks of ROS 2 applications.

Continue to [Lesson 2: Nodes and Topics](./02-nodes-topics.md) →

## Additional Resources

- [Official ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Environment Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [Ubuntu 22.04 Download](https://ubuntu.com/download/desktop)
