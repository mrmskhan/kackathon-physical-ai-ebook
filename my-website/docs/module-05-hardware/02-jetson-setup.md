---
id: jetson-setup
title: "Lesson 2: NVIDIA Jetson Setup"
sidebar_position: 3
---

# Lesson 2: NVIDIA Jetson Setup

Deploy ROS 2 and AI models on NVIDIA Jetson Orin for edge robotics.

## Why Jetson?

- **Onboard AI**: Run Whisper, LLMs, VSLAM locally on robot
- **Low power**: 15-60W vs 300W+ desktop GPU
- **Compact**: Fits inside humanoid robot chassis
- **ROS 2 native**: Official Ubuntu 20.04/22.04 support

## Recommended Hardware

**NVIDIA Jetson Orin Nano Developer Kit** ($499)
- 8GB RAM, 1024 CUDA cores
- Runs Whisper base model + Nav2 + VSLAM
- Best price/performance for humanoid robotics

**Alternatives:**
- Jetson Orin NX ($599): 16GB RAM, more CUDA cores
- Jetson AGX Orin ($1,999): 64GB RAM, maximum performance

## Flashing JetPack

**1. Download SDK Manager**
```bash
# On development workstation (not Jetson)
wget https://developer.nvidia.com/sdk-manager
chmod +x sdkmanager_[version].deb
sudo apt install ./sdkmanager_[version].deb
```

**2. Flash Jetson**
- Connect Jetson to workstation via USB-C
- Put Jetson in recovery mode (hold RECOVERY + press RESET)
- Run SDK Manager, select JetPack 5.1.2 (Ubuntu 20.04) or 6.0 (Ubuntu 22.04)
- Flash OS + CUDA + cuDNN + TensorRT (~30 minutes)

**3. First Boot**
```bash
# On Jetson after flashing
sudo apt update && sudo apt upgrade -y
jtop  # Monitor GPU/CPU usage (install: sudo -H pip install jetson-stats)
```

## Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source in bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Network Configuration

**Connect Jetson to Workstation:**

```bash
# On Jetson, find IP address
ip addr show

# On workstation, add to /etc/hosts
sudo nano /etc/hosts
# Add: 192.168.1.100  jetson-orin

# Test SSH
ssh username@jetson-orin
```

**Set ROS_DOMAIN_ID:**
```bash
# On both workstation and Jetson
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Test communication
# On Jetson:
ros2 topic pub /test std_msgs/String "data: Hello from Jetson"

# On workstation:
ros2 topic echo /test
```

## Performance Optimization

**Enable Max Power Mode:**
```bash
sudo nvpmodel -m 0  # Max performance
sudo jetson_clocks  # Lock clocks to max frequency
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| SDK Manager won't detect Jetson | Check USB cable, re-enter recovery mode |
| ROS 2 nodes can't communicate | Verify ROS_DOMAIN_ID matches on both machines |
| Low performance | Run `sudo jetson_clocks`, check `jtop` for throttling |

## Cost

- Jetson Orin Nano: $499
- Power supply: $30
- microSD card (64GB+): $15
- **Total**: ~$550

## Next Steps

Lesson 3: Integrate RealSense cameras and IMU sensors.
