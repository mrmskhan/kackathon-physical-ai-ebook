---
id: unity-integration
title: "Lesson 4: Unity Integration"
sidebar_position: 5
---

# Lesson 4: Unity Integration

Learn how to integrate Unity 2022.3 LTS with ROS 2 for high-fidelity visualization and human-robot interaction simulations.

## Learning Objectives

- Install Unity 2022.3 LTS and configure the Unity Editor
- Set up ROS-TCP-Connector for Unity-ROS 2 communication
- Create a Unity scene that subscribes to ROS 2 topics
- Visualize robot sensor data (camera, lidar) in Unity

## Prerequisites

import CalloutBox from '@site/src/components/CalloutBox';

<CalloutBox type="prerequisite" title="Before You Begin">

- ROS 2 Humble installed (Module 1)
- Basic understanding of 3D graphics concepts
- Windows 10/11 or Ubuntu 22.04 (Unity supports both)
- 8GB+ RAM, GPU with DirectX 11+ or OpenGL 4.5+

</CalloutBox>

## Why Unity for Robotics?

While Gazebo excels at physics simulation, **Unity** offers:

- **Photorealistic rendering**: HDRP (High Definition Render Pipeline) for lifelike scenes
- **Rich asset ecosystem**: Pre-made 3D models, materials, animations
- **HRI visualization**: Better for human-robot interaction studies
- **VR/AR support**: Integrate with VR headsets for teleoperation
- **Cross-platform**: Deploy to Windows, Linux, macOS, mobile

**Use case**: When visual fidelity matters more than physics accuracy (e.g., user studies, demonstrations, VR training).

## Installation Steps

### Step 1: Install Unity Hub

**Ubuntu 22.04**:
```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

**Windows**:
Download from [Unity Hub Download Page](https://unity.com/download)

### Step 2: Install Unity 2022.3 LTS

1. Open Unity Hub
2. Click **Installs** → **Install Editor**
3. Select **Unity 2022.3 LTS** (Long-Term Support)
4. Choose modules:
   - ✅ Linux Build Support (if on Ubuntu)
   - ✅ Visual Studio (Windows) or Visual Studio Code (Ubuntu)
   - ✅ Documentation

**Installation size**: ~5-8GB

<CalloutBox type="info" title="Why 2022.3 LTS?">

LTS releases receive 2 years of support and are more stable than feature releases. ROS-TCP-Connector is tested against 2022.3 LTS.

</CalloutBox>

### Step 3: Install ROS-TCP-Connector Package

ROS-TCP-Connector enables Unity to communicate with ROS 2 via TCP sockets.

**In Unity**:
1. Create new 3D project (name: "ROS_Unity_Demo")
2. Open **Window** → **Package Manager**
3. Click **+** → **Add package from git URL**
4. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
5. Click **Add**

**Installation time**: 1-2 minutes

### Step 4: Install ROS 2 TCP Endpoint

On your ROS 2 system (Ubuntu):

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

## Configuring ROS-TCP-Connector

### Unity Side Configuration

1. In Unity, go to **Robotics** → **ROS Settings**
2. Set configuration:
   - **ROS IP Address**: Your Ubuntu machine's IP (e.g., `192.168.1.100`)
   - **ROS Port**: `10000` (default)
   - **Protocol**: ROS 2

Find your IP with:
```bash
hostname -I | awk '{print $1}'
```

<CalloutBox type="warning" title="Network Requirements">

Unity and ROS 2 must be on the same network. If running Unity on Windows and ROS 2 on Ubuntu VM, use **Bridged Network** mode in your VM settings.

</CalloutBox>

### Start ROS 2 TCP Endpoint

On Ubuntu:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Expected output**:
```
[INFO] [tcp_server]: Starting server on 0.0.0.0:10000
```

## Creating Your First Unity-ROS Scene

### Step 1: Set Up Scene Hierarchy

1. **Create** → **3D Object** → **Plane** (ground)
2. **Create** → **3D Object** → **Cube** (robot proxy)
3. Position cube at (0, 0.5, 0) - above ground

### Step 2: Create ROS Connection Script

Create C# script `ROSSubscriber.cs`:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class ROSSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        // Connect to ROS 2
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("/cmd_vel");
        ros.Subscribe<TwistMsg>("/cmd_vel", ReceiveMessage);
    }
    
    void ReceiveMessage(TwistMsg msg)
    {
        // Move cube based on ROS 2 /cmd_vel topic
        float linearX = (float)msg.linear.x;
        float angularZ = (float)msg.angular.z;
        
        transform.Translate(Vector3.forward * linearX * Time.deltaTime);
        transform.Rotate(Vector3.up, angularZ * Mathf.Rad2Deg * Time.deltaTime);
        
        Debug.Log($"Received: linear={linearX}, angular={angularZ}");
    }
}
```

**Attach script**:
1. Select Cube in Hierarchy
2. Drag `ROSSubscriber.cs` onto Cube in Inspector

### Step 3: Test Unity-ROS Communication

**Terminal 1 (Ubuntu)**: Start ROS endpoint
```bash
ros2 run ros_tcp_endpoint default_server_endpoint
```

**Terminal 2 (Ubuntu)**: Publish test messages
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

**Unity**: Press **Play** button

**Expected result**: Cube moves forward and rotates in Unity scene.

## Visualizing Sensor Data

### Camera Visualization

Create a ROS 2 camera subscriber in Unity:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine;

public class CameraVisualization : MonoBehaviour
{
    public Renderer imageRenderer;
    private Texture2D texture;
    
    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>("/camera/image", UpdateImage);
    }
    
    void UpdateImage(ImageMsg msg)
    {
        if (texture == null)
        {
            texture = new Texture2D((int)msg.width, (int)msg.height, TextureFormat.RGB24, false);
            imageRenderer.material.mainTexture = texture;
        }
        
        texture.LoadRawTextureData(msg.data);
        texture.Apply();
    }
}
```

**Usage**:
1. Create **Quad** object (display screen)
2. Attach `CameraVisualization` script
3. Assign Quad's Renderer to `imageRenderer` field
4. Publish camera images from ROS 2

## Unity HDRP for Photorealism

For high-fidelity rendering:

1. **Window** → **Package Manager**
2. Install **High Definition RP**
3. **Edit** → **Project Settings** → **Graphics**
4. Assign HDRP asset

**Benefits**:
- Realistic lighting (ray tracing if GPU supports)
- Post-processing effects (depth of field, motion blur)
- Material system with PBR (Physically Based Rendering)

<CalloutBox type="tip" title="Performance vs Quality">

HDRP requires powerful GPUs (GTX 1060+ recommended). For lightweight scenes, stick with Universal Render Pipeline (URP).

</CalloutBox>

## Common Unity-ROS Workflows

### 1. Teleoperation Interface

Unity displays robot camera feed, user controls via keyboard/gamepad, commands sent to ROS 2.

### 2. VR Training Simulator

Unity handles VR headset rendering, ROS 2 simulates robot physics in Gazebo, Unity visualizes in VR.

### 3. Human-Robot Interaction Studies

Unity renders realistic human avatars, ROS 2 runs social navigation algorithms, record user responses.

## Troubleshooting

import TroubleshootingBox from '@site/src/components/TroubleshootingBox';

<TroubleshootingBox
  issue="Connection timeout: Unable to connect to ROS"
  symptom="Unity logs show TCP connection errors"
  cause="Firewall blocking port 10000 or incorrect IP address"
  solution="Ubuntu: sudo ufw allow 10000/tcp or disable firewall temporarily. Windows: Add firewall exception for Unity Editor"
  verification="telnet [ROS_IP] 10000 from Unity machine should connect"
/>

<TroubleshootingBox
  issue="No messages received in Unity"
  symptom="ROS topics publish but Unity script doesn't receive"
  cause="Topic names mismatch or message type incompatibility"
  solution="Verify exact topic names: ros2 topic list and check message type matches Unity import"
  verification="ros2 topic echo /cmd_vel shows messages publishing"
/>

<TroubleshootingBox
  issue="Black textures or rendering artifacts"
  symptom="Materials appear black or incorrectly rendered"
  cause="Shader incompatibility with render pipeline"
  solution="Update materials to URP/HDRP shaders: Edit → Render Pipeline → Upgrade Materials"
  verification="Materials show correctly in Scene view"
/>

## Summary

You've integrated Unity with ROS 2:
- Installed Unity 2022.3 LTS and ROS-TCP-Connector
- Configured bidirectional communication via TCP endpoint
- Created a Unity scene that responds to ROS 2 topics
- Learned to visualize sensor data in Unity

Unity complements Gazebo: use Gazebo for physics-accurate simulation, Unity for photorealistic visualization and HRI.

Next lesson covers **environment building** - creating custom worlds with obstacles, stairs, and navigation challenges.

## Additional Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Learn Tutorials](https://learn.unity.com/)
- [HDRP Documentation](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@latest)
