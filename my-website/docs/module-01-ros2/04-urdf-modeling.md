---
id: urdf-modeling
title: "Lesson 4: URDF Modeling"
sidebar_position: 5
---

# Lesson 4: URDF Modeling

Learn to create robot descriptions using URDF and visualize them in RViz2.

## Learning Objectives

- Understand what URDF is and why it's essential for robotics
- Create a basic humanoid robot model with links and joints
- Define robot geometry, mass properties, and visual elements
- Visualize URDF models in RViz2
- Load URDF into Gazebo (preview for Module 2)

## Prerequisites

import CalloutBox from '@site/src/components/CalloutBox';

<CalloutBox type="prerequisite" title="Before You Begin">

- Completed [Lesson 3: Services and Actions](./03-services-actions.md)
- ROS 2 Humble with `ros-humble-desktop` (includes RViz2)
- Basic understanding of 3D coordinate systems

</CalloutBox>

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based language for describing robot kinematics, dynamics, and visual representation.

### Why URDF Matters

URDF serves as the **single source of truth** for your robot across:
- **Simulation**: Gazebo, Isaac Sim
- **Visualization**: RViz2
- **Motion Planning**: MoveIt 2
- **Control**: ros2_control framework
- **Perception**: Sensor placement and transforms

<CalloutBox type="info" title="Industry Standard">

URDF is used by Boston Dynamics (Spot), Fetch Robotics, Universal Robots, and thousands of research robots worldwide.

</CalloutBox>

## URDF Structure

A URDF model consists of:

1. **Links**: Rigid bodies (torso, arm, leg, wheel)
2. **Joints**: Connections between links (revolute, prismatic, fixed)
3. **Sensors**: Cameras, lidars, IMUs
4. **Plugins**: Gazebo controllers, actuators

### Basic URDF Template

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link (required root) -->
  <link name="base_link">
    <!-- Visual representation -->
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <!-- Collision geometry -->
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>

    <!-- Mass properties -->
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

## Links: Robot Bodies

A **link** represents a rigid body part of the robot.

### Link Components

| Component | Purpose | Required? |
|-----------|---------|-----------|
| `<visual>` | What you see in RViz/Gazebo | No (but recommended) |
| `<collision>` | For physics simulation | No (but needed for Gazebo) |
| `<inertial>` | Mass and inertia properties | No (but needed for dynamics) |

### Geometry Types

```xml
<!-- Box -->
<geometry>
  <box size="length width height"/>
</geometry>

<!-- Cylinder -->
<geometry>
  <cylinder radius="0.05" length="0.3"/>
</geometry>

<!-- Sphere -->
<geometry>
  <sphere radius="0.1"/>
</geometry>

<!-- Mesh (STL, DAE files) -->
<geometry>
  <mesh filename="package://my_robot/meshes/arm.stl" scale="1.0 1.0 1.0"/>
</geometry>
```

<CalloutBox type="tip" title="Visual vs Collision">

**Visual geometry** can be detailed meshes (high polygon count).
**Collision geometry** should be simple primitives (boxes, cylinders) for faster physics.

</CalloutBox>

## Joints: Connecting Links

A **joint** defines how two links move relative to each other.

### Joint Types

| Type | Description | Degrees of Freedom |
|------|-------------|-------------------|
| `fixed` | No movement (welded together) | 0 |
| `revolute` | Rotation around an axis (elbow, knee) | 1 (limited range) |
| `continuous` | Unlimited rotation (wheel) | 1 (unlimited) |
| `prismatic` | Linear sliding (elevator, gripper) | 1 (limited range) |
| `planar` | XY plane movement | 2 |
| `floating` | Free in 3D space (base of mobile robot) | 6 |

### Joint Definition

```xml
<joint name="hip_to_torso" type="fixed">
  <parent link="hip"/>
  <child link="torso"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

### Joint Parameters

| Parameter | Description |
|-----------|-------------|
| `parent` | Fixed link (base of connection) |
| `child` | Moving link |
| `origin xyz` | Position offset from parent (meters) |
| `origin rpy` | Roll-pitch-yaw rotation (radians) |
| `axis xyz` | Rotation/translation axis |
| `limit` | Joint limits (angle, effort, velocity) |

## Creating a Humanoid URDF

Let's build a simple humanoid robot with:
- 1 torso
- 2 legs (upper leg, lower leg, foot)
- 2 arms (upper arm, lower arm, hand)

**File**: `examples/humanoid_basic.urdf` (see full code in [Examples](#hands-on-examples))

### Humanoid Structure

```
base_link (pelvis)
├── torso (fixed joint)
│   ├── left_upper_arm (revolute)
│   │   └── left_lower_arm (revolute)
│   │       └── left_hand (fixed)
│   └── right_upper_arm (revolute)
│       └── right_lower_arm (revolute)
│           └── right_hand (fixed)
├── left_upper_leg (revolute)
│   └── left_lower_leg (revolute)
│       └── left_foot (fixed)
└── right_upper_leg (revolute)
    └── right_lower_leg (revolute)
        └── right_foot (fixed)
```

### Torso Example

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.4 0.6"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.4 0.6"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="15.0"/>
    <inertia ixx="0.5" ixy="0.0" ixz="0.0"
             iyy="0.5" iyz="0.0" izz="0.5"/>
  </inertial>
</link>

<joint name="base_to_torso" type="fixed">
  <parent link="base_link"/>
  <child link="torso"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</joint>
```

### Leg Example

```xml
<!-- Upper leg -->
<link name="left_upper_leg">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.05" ixy="0.0" ixz="0.0"
             iyy="0.05" iyz="0.0" izz="0.05"/>
  </inertial>
</link>

<joint name="left_hip" type="revolute">
  <parent link="base_link"/>
  <child link="left_upper_leg"/>
  <origin xyz="0 0.15 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
</joint>
```

## Visualizing in RViz2

### Step 1: Install Joint State Publisher

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

### Step 2: Launch URDF in RViz2

Create a launch file or use the display helper:

```bash
ros2 launch urdf_tutorial display.launch.py model:=path/to/humanoid_basic.urdf
```

**Or manually**:

```bash
# Terminal 1: Publish robot state
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat humanoid_basic.urdf)"

# Terminal 2: Joint state GUI (move joints interactively)
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: RViz2
rviz2
```

### Step 3: Configure RViz2

1. Click **Add** → **RobotModel**
2. Set **Fixed Frame** to `base_link`
3. You should see your humanoid!

Use the Joint State Publisher GUI to move arms and legs interactively.

<CalloutBox type="tip" title="RViz2 Camera Controls">

- **Left-click + drag**: Rotate view
- **Middle-click + drag**: Pan
- **Scroll**: Zoom in/out

</CalloutBox>

## URDF Best Practices

### 1. Consistent Naming

```xml
<!-- Good -->
<link name="left_upper_arm"/>
<link name="right_upper_arm"/>
<joint name="left_shoulder"/>
<joint name="right_shoulder"/>

<!-- Bad -->
<link name="arm1"/>
<link name="ArmRight"/>
<joint name="j1"/>
```

### 2. Proper Coordinate Frames

ROS convention:
- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up

### 3. Realistic Inertias

Use tools like MeshLab or CAD software to compute accurate inertia tensors for complex shapes.

### 4. Modular with Xacro

For complex robots, use **Xacro** (XML Macros) to avoid repetition:

```xml
<xacro:macro name="leg" params="prefix reflect">
  <link name="${prefix}_upper_leg">
    <!-- Geometry -->
  </link>
  <joint name="${prefix}_hip" type="revolute">
    <origin xyz="0 ${reflect*0.15} 0" rpy="0 0 0"/>
    <!-- ... -->
  </joint>
</xacro:macro>

<!-- Instantiate left and right legs -->
<xacro:leg prefix="left" reflect="1"/>
<xacro:leg prefix="right" reflect="-1"/>
```

## Hands-On Examples

Full URDF code is available in the `examples/` directory:

- **[humanoid_basic.urdf](./examples/humanoid_basic.urdf)** - Complete humanoid model with 11 links

Try modifying the URDF:
1. Add a head link to the torso
2. Change arm/leg lengths
3. Add color materials
4. Adjust joint limits

## Common Issues

import TroubleshootingBox from '@site/src/components/TroubleshootingBox';

<TroubleshootingBox
  issue="RViz2 shows no robot"
  symptom="RobotModel display is added but nothing appears"
  cause="Fixed Frame not set correctly or robot_description topic missing"
  solution={`# Check if robot_description is published
ros2 topic list | grep robot_description

# In RViz2, set Fixed Frame to "base_link"
# Ensure RobotModel is subscribed to /robot_description`}
  verification="ros2 topic echo /robot_description --once"
/>

<TroubleshootingBox
  issue="XML parsing error"
  symptom="Failed to parse URDF file"
  cause="Malformed XML (missing closing tags, invalid characters)"
  solution={`# Validate URDF syntax
check_urdf humanoid_basic.urdf

# If check_urdf is not installed
sudo apt install liburdfdom-tools`}
  verification="check_urdf humanoid_basic.urdf  # Should show tree structure"
/>

<TroubleshootingBox
  issue="Joints don't move in RViz2"
  symptom="Joint State Publisher GUI sliders don't affect robot"
  cause="joint_state_publisher not publishing to /joint_states topic"
  solution={`# Check joint_states topic
ros2 topic echo /joint_states

# Restart joint_state_publisher_gui
ros2 run joint_state_publisher_gui joint_state_publisher_gui`}
/>

## Checking URDF Validity

### Command-Line Validation

```bash
# Check URDF syntax
check_urdf humanoid_basic.urdf

# Convert URDF to Graphviz (visual tree)
urdf_to_graphiz humanoid_basic.urdf
```

**Expected output**:
```
robot name is: humanoid_basic
---------- Successfully Parsed XML ---------------
root Link: base_link has 2 child(ren)
    child(1):  torso
        child(1):  left_upper_arm
            child(1):  left_lower_arm
    ...
```

## URDF in Simulation

Once you have a valid URDF, you can:

1. **Load into Gazebo** (Module 2):
   ```bash
   ros2 launch gazebo_ros spawn_model.launch.py \
     -entity humanoid -file humanoid_basic.urdf
   ```

2. **Use with MoveIt 2** for motion planning (advanced)

3. **Add sensors** (cameras, lidars) for perception tasks

## Summary

In this lesson, you learned:

- ✅ URDF is the standard format for robot descriptions in ROS 2
- ✅ Links represent rigid bodies, joints connect them
- ✅ Visual, collision, and inertial properties are essential for simulation
- ✅ RViz2 visualizes URDF models in 3D
- ✅ Joint types (fixed, revolute, prismatic) define motion constraints
- ✅ Tools like `check_urdf` validate URDF syntax

## Module 1 Complete!

Congratulations! You've completed **Module 1: ROS 2 Foundation**. You now have:

- ✅ A working ROS 2 Humble environment
- ✅ Understanding of nodes, topics, services, and actions
- ✅ Hands-on experience with publishers, subscribers, service/action clients and servers
- ✅ A basic humanoid URDF model

## Next Steps

You're ready for **Module 2: Simulation Mastery** where you'll:
- Load your URDF into Gazebo Garden
- Add sensors (cameras, lidars, IMUs)
- Simulate physics and test robot behaviors
- Export simulations to Unity for high-fidelity visuals

**Module 2 coming soon!**

## Additional Resources

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html)
