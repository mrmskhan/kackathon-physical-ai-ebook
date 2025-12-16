---
id: vslam
title: "Lesson 3: Visual SLAM (VSLAM)"
sidebar_position: 3
---

# Lesson 3: Visual SLAM (VSLAM)

## What is VSLAM?

**Visual Simultaneous Localization and Mapping:**
- Build 3D map from camera images
- Track robot pose in real-time
- No GPS needed (works indoors)

## Isaac ROS VSLAM

NVIDIA's GPU-accelerated VSLAM package:

- **Input**: Stereo camera or RGB-D
- **Output**:
  - 3D point cloud map
  - Robot odometry (pose over time)
  - Loop closure detection
- **Performance**: Runs at 30 FPS on RTX 3080

## Installation

```bash
sudo apt install ros-humble-isaac-ros-visual-slam
```

## Launch VSLAM Node

```bash
# Terminal 1: Start Isaac Sim with stereo camera
# Terminal 2: Launch VSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Visualize in RViz2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam)/share/isaac_ros_visual_slam/rviz/default.rviz
```

See `examples/vslam_launch.py` for custom launch file.

## Input Topics

VSLAM subscribes to:
- `/camera/left/image_raw` (left stereo image)
- `/camera/right/image_raw` (right stereo image)
- `/camera/left/camera_info` (calibration)

**For RGB-D cameras:**
- `/camera/color/image_raw`
- `/camera/depth/image_raw`

## Output Topics

VSLAM publishes:
- `/visual_slam/tracking/odometry` (robot pose)
- `/visual_slam/tracking/vo_pose_covariance` (uncertainty)
- `/visual_slam/vis/map_points` (3D landmarks)
- `/visual_slam/vis/loop_closure_points`

## Mapping Quality Tips

**Good features:**
- Rich textures (posters, patterns)
- Good lighting
- Static environment

**Avoid:**
- Blank walls
- Reflective surfaces (glass, mirrors)
- Motion blur (move slowly)
- Dynamic objects (people moving)

## Example: Map an Office

1. Launch Isaac Sim with office scene
2. Add stereo camera to robot
3. Start VSLAM node
4. Teleop robot slowly around room
5. Save map:
```bash
ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath "{file_path: 'office_map.ply'}"
```

## Visualize in RViz2

- **PointCloud2**: Show 3D map points
- **Odometry**: Show robot trajectory (green line)
- **TF**: Show camera frames

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No map points | Add more textured objects |
| Drift in pose | Enable loop closure |
| Low FPS | Reduce image resolution |

## Next Lesson

Use VSLAM map for autonomous navigation with Nav2.
