---
id: physics-simulation
title: "Lesson 3: Physics Simulation"
sidebar_position: 4
---

# Lesson 3: Physics Simulation

Learn how to configure physics engines, simulate gravity and collisions, and add sensors to your robot models in Gazebo.

## Learning Objectives

- Configure physics engines (DART vs Bullet) for different simulation needs
- Simulate realistic gravity, friction, and contact dynamics
- Add sensor plugins (lidar, camera, IMU) to robot models
- Tune physics parameters for simulation accuracy and performance

## Prerequisites

import CalloutBox from '@site/src/components/CalloutBox';

<CalloutBox type="prerequisite" title="Before You Begin">

- Gazebo Garden installed (Lesson 2)
- Basic understanding of SDF format
- URDF robot model from Module 1 (optional for sensor integration)

</CalloutBox>

## Physics Engines in Gazebo

Gazebo Garden supports multiple physics engines:

| Engine | Best For | Pros | Cons |
|--------|----------|------|------|
| **DART** | Multi-body dynamics, manipulation | Accurate, stable, supports constraints | Slower for large scenes |
| **Bullet** | Fast prototyping, large environments | Fast, good collision detection | Less accurate for complex contacts |
| **TPE** | Simple 2D/3D simulations | Lightweight, fast | Limited features |

**Default**: DART (recommended for robotics)

### Configuring the Physics Engine

In your SDF world file:

```xml
<world name="physics_world">
  <physics name="dart_physics" type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

**Key parameters**:
- `max_step_size`: Physics timestep (0.001 = 1ms, smaller = more accurate but slower)
- `real_time_factor`: 1.0 = real-time, 2.0 = 2x faster, 0.5 = slow motion
- `real_time_update_rate`: Hz (1000 = update every 1ms)

## Simulating Gravity

Gravity affects all objects with mass:

```xml
<world name="gravity_demo">
  <gravity>0 0 -9.81</gravity> <!-- Earth gravity: -9.81 m/s² on Z-axis -->
  
  <model name="falling_box">
    <pose>0 0 5 0 0 0</pose> <!-- Start 5 meters above ground -->
    <link name="box_link">
      <inertial>
        <mass>1.0</mass> <!-- 1 kg mass -->
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.5 0.5 0.5</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.5 0.5 0.5</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</world>
```

**Result**: Box falls from 5m height, accelerating at 9.81 m/s²

<CalloutBox type="tip" title="Testing Different Gravities">

Simulate different environments:
- **Moon**: `0 0 -1.62`
- **Mars**: `0 0 -3.71`
- **Microgravity**: `0 0 -0.01`

</CalloutBox>

## Contact Dynamics: Friction and Restitution

**Friction** resists sliding motion. **Restitution** (bounciness) affects collisions.

### Configuring Surface Properties

```xml
<collision name="ground_collision">
  <geometry>
    <plane><normal>0 0 1</normal></plane>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>   <!-- Coefficient of friction (0 = ice, 1 = rubber) -->
        <mu2>1.0</mu2> <!-- Perpendicular friction -->
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.0</restitution_coefficient> <!-- 0 = no bounce, 1 = perfect bounce -->
    </bounce>
  </surface>
</collision>
```

**Real-world friction values**:
- Ice on ice: 0.02
- Wood on wood: 0.4
- Rubber on concrete: 1.0
- Dry metal on metal: 0.6

### Bouncing Ball Example

```xml
<model name="bouncing_ball">
  <pose>0 0 3 0 0 0</pose>
  <link name="ball_link">
    <inertial>
      <mass>0.2</mass> <!-- 200g ball -->
    </inertial>
    <collision name="collision">
      <geometry>
        <sphere><radius>0.1</radius></sphere>
      </geometry>
      <surface>
        <bounce>
          <restitution_coefficient>0.8</restitution_coefficient> <!-- 80% energy retained -->
        </bounce>
      </surface>
    </collision>
  </link>
</model>
```

**Result**: Ball bounces repeatedly, losing 20% energy per bounce until it settles.

## Adding Sensors to Your Robot

Sensors provide data for perception and navigation. Common sensor types:

### 1. Lidar Sensor

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.5 0 0 0</pose>
  <topic>/lidar</topic>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
    </range>
  </lidar>
</sensor>
```

**Publishes to**: `/lidar` topic (sensor_msgs/LaserScan in ROS 2)

### 2. Camera Sensor

```xml
<sensor name="camera" type="camera">
  <pose>0.2 0 0.3 0 0 0</pose>
  <topic>/camera</topic>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.396</horizontal_fov> <!-- 80 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
</sensor>
```

**Publishes to**: `/camera` topic (sensor_msgs/Image in ROS 2)

### 3. IMU Sensor

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.1 0 0 0</pose>
  <topic>/imu</topic>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></z>
    </linear_acceleration>
  </imu>
</sensor>
```

**Publishes to**: `/imu` topic (sensor_msgs/Imu in ROS 2)

## Sensor Noise and Realism

Real sensors have noise. Add Gaussian noise for realism:

```xml
<noise type="gaussian">
  <mean>0.0</mean>      <!-- Average error (bias) -->
  <stddev>0.01</stddev> <!-- Standard deviation (noise level) -->
</noise>
```

**Why add noise?**
- Simulated data trains AI models to handle real-world imperfections
- Tests robustness of navigation and perception algorithms
- Prevents overfitting to perfect simulated data

<CalloutBox type="warning" title="The Sim-to-Real Gap">

Perfect simulated sensors create unrealistic expectations. Always add noise matching your target hardware's specifications (check sensor datasheets).

</CalloutBox>

## Tuning Physics Performance

Balance accuracy vs speed:

### For High Accuracy (Manipulation, Contact-Rich Tasks)
```xml
<physics type="dart">
  <max_step_size>0.0005</max_step_size> <!-- 0.5ms timestep -->
  <real_time_factor>0.5</real_time_factor> <!-- Run slower for stability -->
</physics>
```

### For Fast Simulation (Navigation, Large Environments)
```xml
<physics type="bullet">
  <max_step_size>0.01</max_step_size> <!-- 10ms timestep -->
  <real_time_factor>2.0</real_time_factor> <!-- Run 2x faster -->
</physics>
```

## Practical Example: Robot with Sensors

Combine concepts in a complete robot model:

```xml
<model name="sensor_robot">
  <link name="chassis">
    <inertial><mass>10.0</mass></inertial>
    <collision name="collision">
      <geometry><box><size>0.5 0.3 0.2</size></box></geometry>
      <surface>
        <friction><ode><mu>0.8</mu></ode></friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.5 0.3 0.2</size></box></geometry>
    </visual>
    
    <!-- Lidar on top -->
    <sensor name="lidar" type="gpu_lidar">
      <pose>0 0 0.15 0 0 0</pose>
      <topic>/lidar</topic>
      <update_rate>10</update_rate>
      <lidar>
        <scan><horizontal><samples>360</samples></horizontal></scan>
        <range><min>0.1</min><max>10.0</max></range>
      </lidar>
    </sensor>
    
    <!-- Camera in front -->
    <sensor name="camera" type="camera">
      <pose>0.25 0 0.05 0 0 0</pose>
      <topic>/camera</topic>
      <update_rate>30</update_rate>
      <camera>
        <image><width>640</width><height>480</height></image>
      </camera>
    </sensor>
    
    <!-- IMU at center -->
    <sensor name="imu" type="imu">
      <topic>/imu</topic>
      <update_rate>100</update_rate>
    </sensor>
  </link>
</model>
```

## Common Issues and Solutions

import TroubleshootingBox from '@site/src/components/TroubleshootingBox';

<TroubleshootingBox
  issue="Robot falls through ground"
  symptom="Model passes through ground plane as if not solid"
  cause="Missing collision geometry or disabled physics"
  solution="Ensure both ground and robot have <collision> tags with valid geometry"
  verification="gz topic -e -t /world/default/stats shows stable poses"
/>

<TroubleshootingBox
  issue="Unstable simulation / objects vibrating"
  symptom="Models jitter or explode unrealistically"
  cause="Timestep too large or stiff constraints"
  solution="Reduce max_step_size to 0.001 or lower and check mass/inertia values are realistic"
  verification="Simulation runs smoothly without sudden movements"
/>

## Summary

You've learned how to configure realistic physics in Gazebo:
- Chose between DART and Bullet physics engines
- Simulated gravity, friction, and collisions
- Added lidar, camera, and IMU sensors with realistic noise
- Tuned physics parameters for accuracy vs performance

Next lesson covers **Unity integration** for high-fidelity visualization and human-robot interaction.

## Additional Resources

- [Gazebo Physics Documentation](https://gazebosim.org/docs/garden/physics)
- [SDF Sensor Reference](http://sdformat.org/spec?elem=sensor)
- [DART Physics Engine](https://dartsim.github.io/)
- [Bullet Physics](https://pybullet.org/wordpress/)
