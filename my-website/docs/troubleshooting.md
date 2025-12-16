---
id: troubleshooting
title: "Global Troubleshooting"
sidebar_position: 99
---

# Global Troubleshooting

Common issues across all modules and their solutions.

## Docusaurus Build Errors

### MDX Parsing Errors

**Symptoms:**
```
[ERROR] MDX compilation failed
Unexpected character '<' before name
```

**Cause:** Less-than symbols (`<`) interpreted as JSX tags in markdown.

**Solution:**
```bash
# Replace < with &lt; in all markdown files
find docs -name "*.md" -exec sed -i 's/<\([0-9]\)/\&lt;\1/g' {} \;

# Example: Change "<5 seconds" to "&lt;5 seconds"
```

### Node Modules Issues

**Symptoms:**
- Build fails with module not found
- `npm run build` crashes

**Solution:**
```bash
cd my-website
rm -rf node_modules .docusaurus build
npm install
npm run build
```

### Port Already in Use

**Symptoms:**
```
[ERROR] Port 3000 is already in use
```

**Solution:**
```bash
# Find and kill process using port 3000
lsof -ti:3000 | xargs kill -9

# Or use different port
npm run start -- --port 3001
```

## Ubuntu Compatibility Issues

### ROS 2 Installation Fails

**Symptoms:**
- `Unable to locate package ros-humble-desktop`
- GPG key errors

**Solution:**
```bash
# Verify Ubuntu version (must be 22.04 for Humble)
lsb_release -a

# Re-add ROS 2 repository
sudo rm /etc/apt/sources.list.d/ros2.list
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install ros-humble-desktop
```

### Gazebo Won't Start

**Symptoms:**
- Black screen in Gazebo
- `[Err] [REST.cc:205] Error in REST request` errors

**Solution:**
```bash
# Clear Gazebo cache
rm -rf ~/.gazebo/models/*

# Reinstall Gazebo
sudo apt remove --purge gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
```

### NVIDIA Driver Issues

**Symptoms:**
- `nvidia-smi` command not found
- Black screen after driver install

**Solution:**
```bash
# Boot with nomodeset (add to GRUB)
# Edit /etc/default/grub:
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash nomodeset"
sudo update-grub
sudo reboot

# After reboot, reinstall drivers
sudo ubuntu-drivers autoinstall
sudo reboot

# Verify
nvidia-smi
```

## Cloud GPU Setup

### AWS EC2 GPU Instance

**Launch g4dn.xlarge ($0.526/hour):**

1. **Create Instance:**
   - AMI: Deep Learning Base AMI (Ubuntu 22.04)
   - Instance Type: g4dn.xlarge (4 vCPU, 16GB RAM, T4 GPU)
   - Storage: 100GB EBS

2. **Install ROS 2:**
```bash
ssh -i your-key.pem ubuntu@ec2-ip
sudo apt update
# Follow Module 1 ROS 2 installation
```

3. **Enable GUI (Optional):**
```bash
sudo apt install ubuntu-desktop
sudo systemctl set-default graphical.target
# Use VNC or X2Go for remote desktop
```

### Google Colab (Free GPU)

**Limitations:**
- No persistent storage
- 12-hour timeout
- No ROS 2 system packages

**Setup:**
```python
# Install ROS 2 in Colab (not recommended, use for AI models only)
!apt install software-properties-common
!add-apt-repository universe
# ... (full ROS 2 install takes 30+ minutes)

# Better: Use Colab only for Whisper/LLM, run ROS 2 locally
```

### Lambda Labs ($0.50/hour)

**Recommended for GPU work:**

1. Create instance with A10 GPU
2. Select Ubuntu 22.04 image
3. SSH and install ROS 2
4. More cost-effective than AWS for AI workloads

## Python Environment Issues

### PyAudio Installation Fails

**Symptoms:**
```
error: portaudio.h: No such file or directory
```

**Solution:**
```bash
sudo apt install portaudio19-dev python3-pyaudio
pip3 install pyaudio
```

### OpenAI Package Version Conflicts

**Symptoms:**
```
ImportError: cannot import name 'OpenAI' from 'openai'
```

**Solution:**
```bash
# Use v1.x API
pip3 install --upgrade openai>=1.0.0

# Update code for new API
from openai import OpenAI
client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
```

### Whisper Model Download Fails

**Symptoms:**
- Stuck at "Downloading model..."
- Connection timeout

**Solution:**
```bash
# Pre-download models
python3 -c "import whisper; whisper.load_model('base')"

# Or specify local path
model = whisper.load_model("base", download_root="/tmp/whisper")
```

## ROS 2 Communication Issues

### Nodes Can't See Each Other

**Symptoms:**
- `ros2 topic list` shows different topics in different terminals
- `ros2 node list` incomplete

**Solution:**
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be same across all terminals

# Set in all terminals
export ROS_DOMAIN_ID=42

# Or add to bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

### DDS Discovery Issues

**Symptoms:**
- Nodes on different machines can't communicate
- Intermittent topic drops

**Solution:**
```bash
# Switch to Cyclone DDS (more reliable than Fast-DDS)
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add to bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

### QoS Compatibility Errors

**Symptoms:**
```
[WARN] New publisher discovered, but could not match QoS
```

**Solution:**
```python
# In your ROS 2 node, use compatible QoS
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    depth=10
)
self.subscription = self.create_subscription(
    Image, '/camera/image_raw', self.callback, qos)
```

## Hardware-Specific Issues

### RealSense Camera Not Working

**Symptoms:**
- `rs-enumerate-devices` shows nothing
- Permission denied errors

**Solution:**
```bash
# Install udev rules
sudo apt install librealsense2-udev-rules

# Add user to video group
sudo usermod -aG video $USER
newgrp video

# Replug camera
```

### Jetson Overheating

**Symptoms:**
- System throttles performance
- `jtop` shows >80°C

**Solution:**
```bash
# Add heatsink + fan
# Enable max fan speed
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'

# Reduce max power mode if needed
sudo nvpmodel -m 2  # 15W mode instead of 25W
```

### Microphone Latency

**Symptoms:**
- 2-3 second delay in Whisper transcription

**Solution:**
```bash
# Reduce buffer size in whisper_ros_node.py
CHUNK = 512  # Instead of 1024
RECORD_SECONDS = 3  # Instead of 5
```

## Build Performance Issues

### Slow Docusaurus Build (&gt;5 minutes)

**Solution:**
```bash
# Clear cache
rm -rf .docusaurus

# Use development build (faster)
npm run start  # Instead of npm run build

# Disable minification for testing
# In docusaurus.config.ts:
future: {
  experimental_faster: {
    swcJsMinimizer: false,
  },
}
```

### ROS 2 colcon Build Failures

**Symptoms:**
- Package not found errors
- Missing dependencies

**Solution:**
```bash
# Update rosdep
rosdep update

# Install all dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build install log
colcon build --symlink-install
```

## Getting More Help

**Official Support:**
- ROS 2 Discourse: https://discourse.ros.org/
- Docusaurus Discord: https://discord.gg/docusaurus
- GitHub Issues: Report bugs in respective repositories

**Community Resources:**
- ROS Answers: https://answers.ros.org/
- Stack Overflow: Tag `ros2`, `gazebo`, `docusaurus`
- Reddit: r/ROS, r/robotics

**Debugging Tips:**
1. Check logs: `~/.ros/log/latest/` for ROS 2 logs
2. Use verbose mode: `ros2 run <pkg> <node> --ros-args --log-level debug`
3. Monitor topics: `ros2 topic hz /topic_name` for frequency
4. Visualize: `rqt_graph` to see node connections
5. Test individually: Isolate components before full integration
