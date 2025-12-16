---
id: troubleshooting
title: "Troubleshooting Guide"
sidebar_position: 4
---

# Troubleshooting Guide

Common issues and solutions for the VLA capstone project.

## Speech Recognition Issues

### Whisper Not Detecting Microphone

**Symptoms:**
- `[ERROR] No microphone found`
- `[ERROR] PortAudio error: Invalid device`

**Solutions:**

1. List available devices:
```bash
python3 -c "import pyaudio; p=pyaudio.PyAudio(); \
  [print(i, p.get_device_info_by_index(i)['name']) \
  for i in range(p.get_device_count())]"
```

2. Update device ID in `whisper_ros_node.py`:
```python
MICROPHONE_INDEX = 1  # Change to your device index
```

3. Test recording:
```bash
arecord -D hw:1,0 -f cd test.wav -d 5
aplay test.wav
```

### Low Transcription Accuracy

**Symptoms:**
- Whisper outputs gibberish
- Commands not recognized

**Solutions:**

1. Reduce background noise
2. Use push-to-talk mode:
```python
# Modify whisper_ros_node.py
def listen_with_push_to_talk(self):
    input("Press Enter to start recording...")
    audio = self.record_audio(duration=5)
    return self.transcribe_audio(audio)
```

3. Upgrade to larger model:
```python
model = whisper.load_model("medium")  # Instead of "base"
```

## LLM Planning Issues

### OpenAI API Key Not Found

**Symptoms:**
- `[ERROR] OpenAI API key not set`
- `openai.error.AuthenticationError`

**Solutions:**

1. Verify environment variable:
```bash
echo $OPENAI_API_KEY
```

2. Set in current session:
```bash
export OPENAI_API_KEY="sk-..."
```

3. Add to bashrc permanently:
```bash
echo 'export OPENAI_API_KEY="sk-..."' >> ~/.bashrc
source ~/.bashrc
```

### LLM Generates Invalid Actions

**Symptoms:**
- `[ERROR] Unknown action: flyyyyy`
- JSON parsing errors

**Solutions:**

1. Improve system prompt:
```python
system_prompt = """CRITICAL: Only output valid JSON.

Valid actions (use EXACTLY these names):
- navigate(kitchen)
- navigate(bedroom)
- pick(cup)
- place(table)

Invalid examples:
- fly(sky) ❌
- teleport(mars) ❌

Output format:
{"actions": ["navigate(kitchen)", "pick(cup)"]}
"""
```

2. Add validation in `llm_planner.py`:
```python
VALID_ACTIONS = ["navigate", "pick", "place", "wait", "speak"]
VALID_LOCATIONS = ["kitchen", "bedroom", "living_room"]

def validate_action(action_str):
    action_name = action_str.split("(")[0]
    if action_name not in VALID_ACTIONS:
        raise ValueError(f"Invalid action: {action_name}")
```

3. Use local Llama for better control (no API costs):
```bash
# Install llama.cpp
git clone https://github.com/ggerganov/llama.cpp
cd llama.cpp && make

# Download model
wget https://huggingface.co/TheBloke/Llama-2-7B-Chat-GGUF/resolve/main/llama-2-7b-chat.Q4_K_M.gguf

# Run local server
./server -m llama-2-7b-chat.Q4_K_M.gguf --host 0.0.0.0 --port 8080
```

## Navigation Issues

### Robot Won't Move in Gazebo

**Symptoms:**
- Nav2 sends goals but robot doesn't move
- `cmd_vel` topic has no data

**Solutions:**

1. Check topic connections:
```bash
ros2 topic list | grep cmd_vel
ros2 topic echo /cmd_vel
```

2. Verify controller is running:
```bash
ros2 node list | grep controller
```

3. Check Gazebo physics:
```bash
# In Gazebo GUI: World → Physics → Real Time Factor
# Should be ~1.0, not 0.0
```

### Navigation Goals Rejected

**Symptoms:**
- `[ERROR] Goal rejected by Nav2`
- Robot doesn't move after LLM planning

**Solutions:**

1. Verify map exists:
```bash
ros2 topic echo /map --once
```

2. Check costmap visualization in RViz2:
```bash
rviz2
# Add → By topic → /global_costmap/costmap → Map
```

3. Ensure goal is on free space:
```python
# In action_orchestrator.py, print goal coordinates
self.get_logger().info(f"Sending goal: x={x}, y={y}")
```

4. Check Nav2 status:
```bash
ros2 topic echo /navigate_to_pose/_action/status
```

### Robot Gets Stuck

**Symptoms:**
- Robot oscillates in place
- Path planning fails repeatedly

**Solutions:**

1. Tune DWB controller parameters:
```yaml
# In nav2_params.yaml
FollowPath:
  max_vel_x: 0.3  # Reduce max speed
  min_vel_x: -0.1  # Allow backward motion
  xy_goal_tolerance: 0.25  # Increase tolerance
  yaw_goal_tolerance: 0.3
```

2. Enable recovery behaviors:
```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: navigate_w_replanning_and_recovery.xml
```

3. Clear costmap manually:
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap
```

## Integration Issues

### Nodes Can't Communicate

**Symptoms:**
- Topics not visible across terminals
- `ros2 topic list` shows different results in different terminals

**Solutions:**

1. Verify ROS_DOMAIN_ID matches:
```bash
# In all terminals
echo $ROS_DOMAIN_ID
```

2. Check network configuration:
```bash
# Disable DDS discovery issues
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

3. Use single launch file (avoids separate terminals):
```bash
ros2 launch vla_demos capstone.launch.py
```

### High Latency (&gt;10 seconds)

**Symptoms:**
- Slow response to voice commands
- Robot delays before moving

**Solutions:**

1. Profile latency at each stage:
```bash
# Measure Whisper latency
ros2 topic echo /voice_command --field data | ts '[%H:%M:%S]'

# Measure LLM latency
ros2 topic echo /action_sequence | ts '[%H:%M:%S]'
```

2. Optimize Whisper:
```python
# Use "tiny" model for speed
model = whisper.load_model("tiny")  # 5x faster than "base"
```

3. Use local LLM (no API latency):
```python
# Replace OpenAI with llama.cpp server
response = requests.post("http://localhost:8080/completion",
    json={"prompt": prompt, "max_tokens": 100})
```

4. Run Nav2 on separate machine:
```bash
# On powerful workstation
export ROS_DOMAIN_ID=42
ros2 launch nav2_bringup navigation_launch.py
```

## Build & Dependency Issues

### Docusaurus Build Fails

**Symptoms:**
- `npm run build` errors
- MDX parsing errors

**Solutions:**

1. Clear cache:
```bash
cd my-website
rm -rf .docusaurus build node_modules
npm install
npm run build
```

2. Fix MDX syntax errors:
```bash
# Replace < with &lt; in markdown
sed -i 's/<\([0-9]\)/\&lt;\1/g' docs/**/*.md
```

### ROS 2 Build Fails

**Symptoms:**
- `colcon build` errors
- Missing dependencies

**Solutions:**

1. Install missing dependencies:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

2. Clean build:
```bash
rm -rf build install log
colcon build --symlink-install
```

## Hardware Deployment Issues

### Jetson Out of Memory

**Symptoms:**
- Whisper crashes
- LLM fails to load

**Solutions:**

1. Use smaller models:
```python
# Whisper: tiny (39M) instead of base (74M)
model = whisper.load_model("tiny")
```

2. Increase swap:
```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

3. Run LLM on workstation, use network:
```python
# On Jetson
LLM_SERVER_URL = "http://192.168.1.100:8080"
```

### RealSense Not Detected

**Symptoms:**
- `rs-enumerate-devices` shows nothing
- ROS 2 camera node fails

**Solutions:**

1. Check USB connection:
```bash
lsusb | grep Intel
# Should show: Intel Corp. RealSense D435i
```

2. Update firmware:
```bash
sudo apt install intel-realsense-dfu
rs-fw-update -l  # List devices
```

3. Verify permissions:
```bash
sudo usermod -aG video $USER
```

## Getting Help

**Resources:**
- ROS 2 Discourse: https://discourse.ros.org/
- Nav2 GitHub Issues: https://github.com/ros-planning/navigation2/issues
- Whisper GitHub: https://github.com/openai/whisper

**Debugging Checklist:**
- [ ] Check all terminals for error messages
- [ ] Verify `ros2 topic list` shows all expected topics
- [ ] Run `ros2 node list` to confirm all nodes running
- [ ] Check `rqt_graph` for broken connections
- [ ] Review launch file for typos
- [ ] Test components individually before full integration
