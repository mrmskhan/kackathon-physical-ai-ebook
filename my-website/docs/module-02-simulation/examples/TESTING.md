# Module 2 Code Examples - Testing Guide

This document provides step-by-step instructions to test and validate all Module 2 code examples.

## Prerequisites

Before testing, ensure you have:

- Ubuntu 22.04 with ROS 2 Humble installed
- Gazebo Garden installed (`gz-garden`)
- ros_gz packages installed (`ros-humble-ros-gz`)
- Unity 2022.3 LTS (for Unity examples)
- ROS-TCP-Connector package (for Unity examples)

## Test 1: simple_world.sdf

**Purpose**: Verify basic Gazebo world loads with obstacles and physics.

**Test Commands**:
```bash
cd docs/module-02-simulation/examples
gz sim simple_world.sdf
```

**Expected Results**:
- ✓ Gazebo GUI opens without errors
- ✓ Ground plane visible (gray 100x100m)
- ✓ Directional sun lighting active
- ✓ Red box obstacle at (2, 0, 0.5)
- ✓ Green box obstacle at (-2, 2, 0.5) rotated 45°
- ✓ Blue sphere spawns at (0, 0, 2) and falls due to gravity
- ✓ Sphere bounces with restitution coefficient 0.7
- ✓ Physics engine shows DART running at 1000 Hz

**Validation Checklist**:
- [ ] World loads without SDF parsing errors
- [ ] All 4 models visible in scene tree (ground, 2 boxes, sphere)
- [ ] Sphere exhibits realistic physics (fall + bounce)
- [ ] Frame rate stable (no lag)

---

## Test 2: spawn_humanoid.launch.py

**Purpose**: Verify ROS 2 launch file spawns humanoid robot in Gazebo.

**Prerequisites**:
- Humanoid URDF from Module 1 (`humanoid_basic.urdf`)
- Update line 90 with correct URDF path

**Test Commands**:
```bash
# Terminal 1: Source ROS 2
source /opt/ros/humble/setup.bash
cd my-website/docs/module-02-simulation/examples

# Update URDF path in spawn_humanoid.launch.py (line 90 and 124)
# Then launch:
ros2 launch spawn_humanoid.launch.py world:=simple_world.sdf
```

**Expected Results**:
- ✓ Gazebo launches with simple_world.sdf
- ✓ Humanoid robot spawns at (0, 0, 0.5)
- ✓ Robot state publisher node running
- ✓ ROS-Gazebo bridges active for /clock and /pose
- ✓ TF tree published for robot links

**Validation Checklist**:
- [ ] `ros2 node list` shows robot_state_publisher
- [ ] `ros2 topic list` shows /clock and /model/humanoid/pose
- [ ] `ros2 topic echo /clock` outputs simulation time
- [ ] Robot visible and upright in Gazebo
- [ ] No URDF parsing errors in console

---

## Test 3: sensors.xacro

**Purpose**: Verify sensor macros generate valid Gazebo plugins.

**Test Setup**:
1. Create test URDF that includes sensors.xacro
2. Add sensor macros to a base link

**Test URDF** (`test_sensors.urdf.xacro`):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_robot">

  <xacro:include filename="sensors.xacro"/>

  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.5 0.5"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.5 0.5 0.5"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <xacro:lidar_sensor parent="base_link">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </xacro:lidar_sensor>

  <xacro:camera_sensor parent="base_link">
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
  </xacro:camera_sensor>

  <xacro:imu_sensor parent="base_link">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </xacro:imu_sensor>

</robot>
```

**Test Commands**:
```bash
# Process xacro to URDF
xacro test_sensors.urdf.xacro > test_sensors.urdf

# Check for errors
check_urdf test_sensors.urdf

# Spawn in Gazebo
gz sim -r -v 4 test_sensors.urdf
```

**Expected Results**:
- ✓ Xacro processing completes without errors
- ✓ URDF validation passes
- ✓ Robot spawns with 3 sensors attached
- ✓ `/lidar` topic publishes LaserScan messages at 10 Hz
- ✓ `/camera/image` topic publishes Image messages at 30 Hz
- ✓ `/imu` topic publishes IMU messages at 100 Hz
- ✓ Lidar visualization shows red rays in Gazebo

**Validation Checklist**:
- [ ] `ros2 topic list | grep -E '(lidar|camera|imu)'` shows all 3 topics
- [ ] `ros2 topic hz /lidar` reports ~10 Hz
- [ ] `ros2 topic hz /camera/image` reports ~30 Hz
- [ ] `ros2 topic hz /imu` reports ~100 Hz
- [ ] `ros2 topic echo /lidar --once` shows 360 range samples
- [ ] Lidar rays visible in Gazebo GUI (toggle visualization)

---

## Test 4: UnitySceneSetup.cs

**Purpose**: Verify Unity scene connects to ROS 2 and receives messages.

**Prerequisites**:
- Unity 2022.3 LTS project created
- ROS-TCP-Connector package imported
- Empty GameObject with UnitySceneSetup.cs attached

**Test Setup**:
1. Open Unity Editor
2. Create new scene
3. Import ROS-TCP-Connector package:
   - Window → Package Manager
   - Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Create empty GameObject, attach `UnitySceneSetup.cs`
5. In Inspector, set ROS IP to your Ubuntu machine's IP
6. Create a Cube GameObject and assign it to `robotObject` field

**Test Commands** (Ubuntu terminal):
```bash
# Terminal 1: Start ROS-TCP-Endpoint
source /opt/ros/humble/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: Publish test velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}" -r 10
```

**In Unity**:
- Press Play
- Observe console logs and robot movement

**Expected Results**:
- ✓ Unity console shows "Connecting to ROS 2 at [IP]:10000"
- ✓ Unity console shows "Subscribed to /cmd_vel"
- ✓ Green "ROS 2: Connected" label in top-left corner
- ✓ Robot cube moves forward (Z-axis) at 1.0 m/s
- ✓ Robot cube rotates at 0.5 rad/s
- ✓ Console logs velocity values every frame

**Validation Checklist**:
- [ ] No connection errors in Unity console
- [ ] ROS-TCP-Endpoint logs show Unity connection
- [ ] Robot GameObject translates and rotates smoothly
- [ ] `Debug.Log` shows parsed velocity values
- [ ] Connection survives Unity Play/Stop cycles

---

## Test 5: test_arena.sdf

**Purpose**: Verify navigation test arena loads with all challenge elements.

**Test Commands**:
```bash
cd docs/module-02-simulation/examples
gz sim test_arena.sdf
```

**Expected Results**:
- ✓ 10x10 meter enclosed arena with 4 boundary walls
- ✓ Doorway frame (1.2m wide opening) at position (2, 0.6, 0)
- ✓ 3-step staircase at (-2, -2, 0) with 0.2m rise per step
- ✓ Ramp at (-2, 2, 0) with 11° incline
- ✓ Three obstacles:
  - Red box (0.8×0.8×1m) at (0, -2, 0.5)
  - Green box (0.6×0.6×0.6m) rotated at (3, 2.5, 0.3)
  - Blue cylinder (r=0.3, h=0.5) at (-3.5, 0.5, 0.25)
- ✓ DART physics engine active
- ✓ Proper lighting (directional sun + ambient)

**Validation Checklist**:
- [ ] World loads without errors
- [ ] All 10 models present in scene tree:
  - ground, 4 walls, doorway (3 links), stairs (3 links), ramp, 3 obstacles
- [ ] Doorway opening exactly 1.2m (measure in Gazebo)
- [ ] Stairs form ascending steps (0.2m, 0.4m, 0.6m heights)
- [ ] Ramp angle measures ~11° (pitch = 0.19 rad)
- [ ] All obstacles static (no drift or falling)
- [ ] Physics stable at 1000 Hz update rate

**Navigation Testing** (optional with robot):
- [ ] Robot can navigate through doorway without collision
- [ ] Robot cannot climb stairs (too steep for wheeled robot)
- [ ] Robot can use ramp as alternative path
- [ ] Robot collision detection works on all obstacles

---

## Overall Module 2 Validation

### Syntax Validation (Automated)

```bash
# Validate all SDF files
for f in *.sdf; do
  echo "Validating $f..."
  gz sdf -k "$f" || echo "ERROR: $f has syntax errors"
done

# Validate Python launch file
python3 -m py_compile spawn_humanoid.launch.py

# Validate xacro file
xacro --check-order sensors.xacro
```

### Integration Tests

**Test 1**: Spawn humanoid in test_arena
```bash
ros2 launch spawn_humanoid.launch.py world:=test_arena.sdf x_pose:=0.0 y_pose:=0.0 z_pose:=0.5
```
- Robot should spawn in center of arena
- All navigation challenges visible around robot

**Test 2**: Add sensors to humanoid and test in test_arena
1. Modify humanoid URDF to include sensors.xacro
2. Spawn in test_arena
3. Verify lidar detects walls, doorway, obstacles
4. Verify camera shows arena environment

### Performance Benchmarks

- Gazebo real-time factor: >0.95 (should match real-time)
- simple_world.sdf load time: &lt;3 seconds
- test_arena.sdf load time: &lt;5 seconds
- Unity ROS connection latency: &lt;50ms
- Sensor topic frequencies within ±5% of configured rates

---

## Troubleshooting

### Gazebo Issues

**Symptom**: "Unable to find uri[model://...]"
- **Fix**: Set `GZ_SIM_RESOURCE_PATH` to include model directories

**Symptom**: Black screen in Gazebo
- **Fix**: Update graphics drivers, check GPU compatibility

**Symptom**: Physics unstable (objects jittering)
- **Fix**: Reduce `max_step_size` in physics settings

### ROS 2 Bridge Issues

**Symptom**: Topics not visible in `ros2 topic list`
- **Fix**: Check ros_gz_bridge node is running, verify bridge arguments

**Symptom**: QoS mismatch warnings
- **Fix**: Ensure publisher/subscriber QoS settings match

### Unity Issues

**Symptom**: "ROS 2: Disconnected" in Unity
- **Fix**: Verify ROS IP address, check firewall, ensure ROS-TCP-Endpoint running

**Symptom**: Robot not moving despite velocity commands
- **Fix**: Check `subscribeToCmdVel` is enabled, verify `robotObject` assigned

**Symptom**: Camera texture not updating
- **Fix**: Assign RawImage component, check image encoding format (RGB24)

---

## Test Results Summary

| Test | Status | Notes |
|------|--------|-------|
| simple_world.sdf | ⬜ Not Tested | |
| spawn_humanoid.launch.py | ⬜ Not Tested | |
| sensors.xacro | ⬜ Not Tested | |
| UnitySceneSetup.cs | ⬜ Not Tested | |
| test_arena.sdf | ⬜ Not Tested | |

**Testing Status**: Ready for validation
**Tested By**: _________
**Test Date**: _________
**Build Version**: Docusaurus 3.x, ROS 2 Humble, Gazebo Garden, Unity 2022.3 LTS

---

## Notes

All code examples are designed to work with:
- ROS 2 Humble (Ubuntu 22.04)
- Gazebo Garden (gz-sim7)
- Unity 2022.3 LTS
- ROS-TCP-Connector v0.7.0+

Update package versions and paths according to your specific installation.
