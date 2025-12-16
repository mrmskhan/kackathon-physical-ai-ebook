---
id: robot-platforms
title: "Lesson 4: Humanoid Robot Platforms"
sidebar_position: 5
---

# Lesson 4: Humanoid Robot Platforms

Deploy on real humanoid robots or use simulation proxies.

## Real Humanoid Platforms

### Unitree Go2 (Quadruped, Humanoid Proxy)

**Cost**: $1,600-$2,700
**Why**: Affordable, ROS 2 SDK, stable locomotion, used as humanoid proxy

**ROS 2 Integration:**
```bash
# Clone Unitree ROS 2 SDK
git clone https://github.com/unitreerobotics/unitree_ros2
cd unitree_ros2
colcon build
source install/setup.bash

# Launch robot interface
ros2 launch unitree_go2_bringup bringup.launch.py
```

**Topics:**
- `/cmd_vel` - Send velocity commands
- `/joint_states` - Read joint positions
- `/imu` - IMU data

### ROBOTIS OP3

**Cost**: $15,000
**Why**: True humanoid, 20 DOF, open-source, ROS 2 native

**Setup:**
```bash
sudo apt install ros-humble-op3-*
ros2 launch op3_bringup op3_bringup.launch.py
```

### Other Platforms

| Platform | Cost | DOF | Best For |
|----------|------|-----|----------|
| **Unitree H1** | $90,000 | 25 | Research labs, advanced bipedal |
| **PAL Robotics TALOS** | $120,000+ | 32 | Industrial, full-size humanoid |
| **Trossen WidowX** | $1,500 | 6 (arm only) | Manipulation tasks |

## Simulation Proxies (If No Robot)

**Use Gazebo models as proxies:**

```bash
# Spawn humanoid model from Module 2
ros2 launch gazebo_humanoid spawn_humanoid.launch.py

# Control via same topics as real robot
ros2 topic pub /cmd_vel geometry_msgs/Twist ...
```

**Benefits:**
- Zero hardware cost
- Safer for testing
- Iterate faster

## Integration Workflow

**1. Network Setup**
```bash
# On robot/Jetson
export ROS_DOMAIN_ID=42

# On workstation
export ROS_DOMAIN_ID=42

# Verify communication
ros2 topic list  # Should see robot topics
```

**2. Sensor Fusion**
```bash
# Combine robot IMU + RealSense
sudo apt install ros-humble-robot-localization
ros2 launch robot_localization ekf.launch.py
```

**3. Deploy VLA Pipeline**
```bash
# Copy Module 4 capstone to Jetson
scp -r vla_demos username@jetson-orin:~/ros2_ws/src/

# On Jetson, build and launch
cd ~/ros2_ws
colcon build --packages-select vla_demos
ros2 launch vla_demos capstone_demo.launch.py use_sim_time:=false
```

## Safety Considerations

- **Emergency Stop**: Hardware kill switch required
- **Geofencing**: Limit movement area in Nav2 params
- **Velocity Limits**: Set conservative `max_vel_x`, `max_vel_theta`
- **Manual Override**: Keep remote control ready

**Example Safe Nav2 Params:**
```yaml
max_vel_x: 0.3  # 30 cm/s max speed
max_vel_theta: 0.5  # Slow turns
robot_radius: 0.4  # Conservative collision buffer
```

## Cost Comparison

| Setup | Cost | Use Case |
|-------|------|----------|
| **Simulation Only** | $0 | Learning, prototyping |
| **Workstation + RealSense** | $2,000 | Sensor development |
| **Workstation + Jetson + Unitree** | $4,000 | Full deployment |
| **Research Lab Setup** | $20,000+ | Advanced humanoids |

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot doesn't respond | Check ROS_DOMAIN_ID, network connectivity |
| Jerky motion | Tune PID gains, check control loop rate |
| Battery drains fast | Reduce max velocity, optimize path planning |
| Topics not visible | Verify DDS settings match (Fast-DDS, Cyclone DDS) |

## Next Steps

Deploy your Module 4 capstone on real hardware. Start with stationary tests (voice commands → LLM → print actions), then enable navigation in safe environment.
