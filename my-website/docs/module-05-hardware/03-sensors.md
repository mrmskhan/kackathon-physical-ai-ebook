---
id: sensors
title: "Lesson 3: Sensor Integration"
sidebar_position: 4
---

# Lesson 3: Sensor Integration

Connect cameras, depth sensors, and IMUs to ROS 2.

## Intel RealSense D435i

**Why RealSense?**
- RGB + stereo depth + IMU in one device
- Native ROS 2 drivers
- USB 3.0 plug-and-play
- Used in most humanoid robots

**Cost**: $329 (D435i), $199 (D435 without IMU)

## Installation

**1. Install librealsense**
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

**2. Verify Camera Detected**
```bash
rs-enumerate-devices  # Should show D435i
```

**3. Launch ROS 2 Node**
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true
```

**4. Visualize in RViz2**
```bash
rviz2
# Add PointCloud2 display, subscribe to /camera/depth/color/points
# Add Image display, subscribe to /camera/color/image_raw
```

## Sensor Data Flow

```mermaid
graph TD
    A[RealSense D435i] -->|USB 3.0| B[Jetson/Workstation]
    B --> C[/camera/color/image_raw]
    B --> D[/camera/depth/image_rect_raw]
    B --> E[/camera/imu]
    C --> F[VSLAM Node]
    D --> F
    E --> G[Robot Localization]
    F --> H[Nav2 Costmap]
```

## IMU Integration (If Separate)

**Common IMUs:**
- BNO055 ($35): I2C/UART, ROS 2 drivers available
- MPU6050 ($5): Cheap, requires custom driver
- Bosch BMI088 ($15): High performance

**ROS 2 IMU Driver Example:**
```bash
sudo apt install ros-humble-bno055
ros2 run bno055 bno055_node

# Verify data
ros2 topic echo /imu/data
```

## Microphone (For Whisper)

**USB Microphone:**
```bash
# Test with arecord
arecord -l  # List devices
arecord -D hw:1,0 -f cd test.wav  # Record 5 seconds

# Use with Whisper ROS node from Module 4
ros2 run vla_demos whisper_ros_node.py --device hw:1,0
```

**Recommended**: Blue Yeti ($100) or Samson Meteor ($70)

## Sensor Calibration

**Camera Intrinsics (Usually Pre-Calibrated):**
```bash
# If needed, use camera_calibration
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/camera/color/image_raw
```

**Camera-IMU Extrinsics:**
```bash
# Use Kalibr for camera-IMU calibration
# https://github.com/ethz-asl/kalibr
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| RealSense not detected | Check USB 3.0 cable, run `rs-enumerate-devices` |
| Low FPS (&lt;30) | Reduce resolution, disable infrared streams |
| IMU drift | Calibrate magnetometer, enable gyro bias correction |
| Microphone not found | Check `arecord -l`, adjust device ID in launch file |

## Cost Breakdown

- RealSense D435i: $329
- USB microphone: $70
- (Optional) External IMU: $35
- **Total**: $400-$450

## Next Steps

Lesson 4: Interface with humanoid robot platforms.
