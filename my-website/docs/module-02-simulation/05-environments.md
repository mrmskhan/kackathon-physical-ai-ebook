---
id: environments
title: "Lesson 5: Environment Building"
sidebar_position: 6
---

# Lesson 5: Environment Building

Learn how to create custom Gazebo worlds with obstacles, stairs, doorways, and navigation challenges for testing physical AI systems.

## Learning Objectives

- Create custom SDF world files from scratch
- Build obstacle courses for navigation testing
- Add interactive elements (doors, stairs, ramps)
- Design test arenas for specific robot capabilities

## Prerequisites

import CalloutBox from '@site/src/components/CalloutBox';

<CalloutBox type="prerequisite" title="Before You Begin">

- Gazebo Garden installed and configured (Lesson 2)
- Understanding of SDF format and physics (Lesson 3)
- Basic 3D geometry concepts (boxes, cylinders, planes)

</CalloutBox>

## SDF World File Structure

A Gazebo world file (.sdf) defines the complete simulation environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="custom_world">
    <!-- Lighting -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
    </light>
    
    <!-- Physics engine -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Ground plane -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Key sections**:
- `<light>`: Illumination (directional sun, point lights, spotlights)
- `<physics>`: Engine configuration
- `<model>`: World objects (ground, obstacles, props)
- `<plugin>`: Dynamic behaviors (sensors, controllers)

## Building Basic Obstacles

### Box Obstacles

```xml
<model name="box_obstacle">
  <pose>2 0 0.5 0 0 0</pose> <!-- X Y Z Roll Pitch Yaw -->
  <static>true</static> <!-- Fixed in place -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box> <!-- Width Depth Height -->
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient> <!-- Red color -->
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Cylinder Pillars

```xml
<model name="pillar">
  <pose>-2 2 1 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder><radius>0.3</radius><length>2</length></cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder><radius>0.3</radius><length>2</length></cylinder>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient> <!-- Gray -->
      </material>
    </visual>
  </link>
</model>
```

### Wall Segments

```xml
<model name="wall">
  <pose>0 5 0.5 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>10 0.2 1</size></box> <!-- Long thin wall -->
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>10 0.2 1</size></box>
      </geometry>
      <material>
        <ambient>0.7 0.6 0.5 1</ambient> <!-- Beige -->
      </material>
    </visual>
  </link>
</model>
```

## Creating Navigation Challenges

### Doorway

```xml
<model name="doorway">
  <static>true</static>
  
  <!-- Left door frame -->
  <link name="frame_left">
    <pose>3 0 1 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>0.2 0.1 2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.2 0.1 2</size></box></geometry>
      <material><ambient>0.3 0.2 0.1 1</ambient></material>
    </visual>
  </link>
  
  <!-- Right door frame -->
  <link name="frame_right">
    <pose>3 1.2 1 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>0.2 0.1 2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.2 0.1 2</size></box></geometry>
      <material><ambient>0.3 0.2 0.1 1</ambient></material>
    </visual>
  </link>
  
  <!-- Top frame -->
  <link name="frame_top">
    <pose>3 0.6 2 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>0.2 1.4 0.1</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.2 1.4 0.1</size></box></geometry>
      <material><ambient>0.3 0.2 0.1 1</ambient></material>
    </visual>
  </link>
</model>
```

**Doorway width**: 1.2m (standard wheelchair accessible)

### Stairs

```xml
<model name="stairs">
  <static>true</static>
  
  <!-- Step 1 -->
  <link name="step1">
    <pose>5 0 0.1 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>1 2 0.2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>1 2 0.2</size></box></geometry>
    </visual>
  </link>
  
  <!-- Step 2 -->
  <link name="step2">
    <pose>5.5 0 0.3 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>1 2 0.2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>1 2 0.2</size></box></geometry>
    </visual>
  </link>
  
  <!-- Step 3 -->
  <link name="step3">
    <pose>6 0 0.5 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>1 2 0.2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>1 2 0.2</size></box></geometry>
    </visual>
  </link>
</model>
```

**Stair dimensions**: 0.2m rise, 0.5m run (shallow stairs for humanoid testing)

### Ramp

```xml
<model name="ramp">
  <static>true</static>
  <link name="link">
    <pose>8 0 0.5 0 0.2 0</pose> <!-- Pitched at 0.2 radians (11 degrees) -->
    <collision name="collision">
      <geometry><box><size>3 2 0.1</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>3 2 0.1</size></box></geometry>
      <material><ambient>0.6 0.6 0.6 1</ambient></material>
    </visual>
  </link>
</model>
```

## Test Arena Template

A complete navigation test arena:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="test_arena">
    <gravity>0 0 -9.81</gravity>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
    </light>
    
    <!-- Physics -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Ground (10x10 meters) -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Boundary walls -->
    <model name="wall_north">
      <pose>0 5 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.1 1</size></box></geometry>
        </visual>
      </link>
    </model>
    
    <model name="wall_south">
      <pose>0 -5 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.1 1</size></box></geometry>
        </visual>
      </link>
    </model>
    
    <!-- Interior obstacles -->
    <include>
      <uri>model://box_obstacle</uri>
      <pose>2 2 0.5 0 0 0</pose>
    </include>
    
    <include>
      <uri>model://doorway</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Using Model Includes

Reuse models across worlds:

1. **Create model directory**:
```bash
mkdir -p ~/.gazebo/models/my_obstacle
```

2. **Create model.config**:
```xml
<?xml version="1.0"?>
<model>
  <name>My Obstacle</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>Custom obstacle model</description>
</model>
```

3. **Create model.sdf** with `<model>` definition

4. **Include in world**:
```xml
<include>
  <uri>model://my_obstacle</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

<CalloutBox type="tip" title="Model Libraries">

Browse pre-made models:
- Gazebo Fuel: [https://app.gazebosim.org/fuel/models](https://app.gazebosim.org/fuel/models)
- Download with: `gz fuel download -u [model_url]`

</CalloutBox>

## Testing Your Environment

Launch your custom world:

```bash
gz sim my_arena.sdf
```

**Validation checklist**:
- ✅ All objects visible in 3D viewport
- ✅ Physics enabled (drop test object, should fall)
- ✅ Collisions work (objects don't pass through walls)
- ✅ Lighting adequate (no pitch-black areas)
- ✅ Performance acceptable (>30 FPS with robot loaded)

## Practical Arena Designs

### 1. Corridor Navigation
Long narrow hallways with turns for testing line-following and collision avoidance.

### 2. Furniture Maze
Living room scenario with tables, chairs, sofas for domestic robot testing.

### 3. Outdoor Terrain
Uneven ground, slopes, rocks for legged robot stability testing.

### 4. Multi-Floor Building
Stairs connecting floors for vertical mobility testing.

## Common Issues and Solutions

import TroubleshootingBox from '@site/src/components/TroubleshootingBox';

<TroubleshootingBox
  issue="Objects fall through ground"
  symptom="Models placed on ground plane immediately fall"
  cause="Ground plane lacks collision geometry"
  solution="Ensure ground model has <collision> tag with valid plane geometry"
  verification="Drop a box from height 2m, should bounce and rest on ground"
/>

<TroubleshootingBox
  issue="World file fails to load"
  symptom="gz sim shows parsing errors"
  cause="Invalid XML or SDF syntax"
  solution="Validate XML: xmllint --noout my_world.sdf and check SDF version matches Gazebo Garden (1.8+)"
  verification="gz sim my_world.sdf loads without errors"
/>

## Summary

You've learned how to build custom simulation environments:
- Created SDF world files with lights, physics, and models
- Built navigation obstacles (walls, doorways, stairs, ramps)
- Designed test arenas for specific robot capabilities
- Reused models via includes for modular world building

These environments provide controlled test scenarios for the AI navigation and perception systems you'll develop in Module 3.

## Additional Resources

- [SDF Specification](http://sdformat.org/spec)
- [Gazebo Fuel Model Library](https://app.gazebosim.org/fuel)
- [Building Robot Simulations (Book)](https://www.packtpub.com/product/robot-operating-system-ros-for-absolute-beginners/9781801818285)
- [Gazebo World Examples](https://github.com/gazebosim/gz-sim/tree/main/examples/worlds)
