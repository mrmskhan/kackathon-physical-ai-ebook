# Quickstart Guide: Physical AI & Humanoid Robotics E-Book

**Purpose**: Get contributors and learners up and running quickly
**Audience**: Content writers, code example contributors, learners setting up environment
**Last Updated**: 2025-12-13

## For Contributors (Content Writers)

### Prerequisites
- Git installed and configured
- Node.js 18+ and npm installed
- Text editor (VS Code recommended with MDX extension)
- Basic familiarity with Markdown/MDX syntax

### Initial Setup

```bash
# 1. Clone the repository
git clone https://github.com/yourusername/hackathon-book-proj.git
cd hackathon-book-proj

# 2. Install Docusaurus dependencies
cd my-website
npm install

# 3. Start local development server
npm run start
```

The e-book will open at `http://localhost:3000`. Changes to `.md` or `.mdx` files hot-reload automatically.

### Content Structure

```
my-website/docs/
├── intro.md              # E-Book intro (edit this for overview)
├── module-01-ros2/       # Module 1: ROS 2 Foundation
│   ├── index.md          # Module overview
│   ├── 01-*.md           # Lessons (numbered)
│   ├── examples/         # Code examples
│   └── sources.md        # APA citations
├── module-02-simulation/ # Module 2: Gazebo & Unity
├── module-03-isaac-ai/   # Module 3: NVIDIA Isaac Sim
├── module-04-vla/        # Module 4: Vision-Language-Action
├── module-05-hardware/   # Module 5: Hardware Deployment
└── capstone/             # Capstone Project
```

### Writing a New Lesson

1. **Create file** in appropriate module directory (e.g., `my-website/docs/module-01-ros2/05-new-lesson.md`)

2. **Add frontmatter** (MDX metadata):
```markdown
---
id: new-lesson
title: New Lesson Title
sidebar_position: 5
---
```

3. **Write content** following template:
```markdown
# New Lesson Title

## Learning Objectives
- [Action verb] [what learner will do]
- [Action verb] [what learner will achieve]

## Prerequisites
- Completed Lesson X
- Installed [package name]

## Concept Explanation
[Clear explanation at grade 9-12 level]

## Example Code
\`\`\`python
# Example with comments
import rclpy
# ...
\`\`\`

## Try It Yourself
1. Step-by-step instructions
2. Expected output

## Troubleshooting
- **Symptom**: Error message
  - **Solution**: Fix steps

## Summary
[Key takeaways]

## Next Steps
[Link to next lesson]
```

4. **Validate** before committing:
```bash
# Build without errors
npm run build

# Check readability (optional, requires external tool)
# Target: Flesch-Kincaid grade 9-12
```

### Adding Code Examples

1. **Create file** in module's `examples/` directory
2. **Test on Ubuntu 22.04** (VM, Docker, or native)
3. **Document** in lesson with:
   - Prerequisites (exact package names)
   - Setup commands
   - Execution command
   - Expected output (screenshot or terminal text)

**Example**:
```markdown
## Example: Simple Publisher

**File**: `examples/publisher.py`

**Prerequisites**:
\`\`\`bash
sudo apt install ros-humble-rclpy python3
\`\`\`

**Setup**:
\`\`\`bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src/examples
cd ~/ros2_ws/src/examples
\`\`\`

**Run**:
\`\`\`bash
python3 publisher.py
\`\`\`

**Expected Output**:
\`\`\`
Publishing: 'Hello World: 0'
Publishing: 'Hello World: 1'
...
\`\`\`

**Code**:
\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    # ... (full code)
\`\`\`
```

### Adding Citations

Edit `sources.md` in each module directory:

```markdown
# Sources: Module 1 - ROS 2 Foundation

## Official Documentation

Open Robotics. (2023). ROS 2 Humble Concepts - Nodes. https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html

Open Robotics. (2023). ROS 2 Tutorials - Writing a Simple Publisher and Subscriber (Python). https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

## Technical Sources

Robotics Back-End. (2024). ROS 2 Tutorial for Beginners. https://roboticsbackend.com/ros2-tutorial-for-beginners/

## Research Papers

(If applicable, full APA citation with DOI)
```

**Requirements**:
- Minimum 10 official sources per module
- Minimum 5 technical/research sources per module
- APA 7th edition format
- Version-specific URLs when possible

### Using Mermaid Diagrams

Mermaid diagrams render natively in Docusaurus:

```markdown
## System Architecture

\`\`\`mermaid
flowchart LR
    Publisher[Publisher Node] -->|messages| Topic[/chatter Topic]
    Topic --> Subscriber[Subscriber Node]
\`\`\`
```

Common diagram types:
- `flowchart LR` - Left-to-right flowchart
- `sequenceDiagram` - Sequence/interaction diagrams
- `classDiagram` - Class relationships
- `stateDiagram-v2` - State machines

---

## For Learners (Environment Setup)

### Module 1: ROS 2 Foundation

**Prerequisites**:
- Ubuntu 22.04 LTS (native install, VM, or dual-boot)
- 4GB+ RAM, 20GB free disk space
- Internet connection for package downloads

**Installation**:
```bash
# 1. Update system
sudo apt update && sudo apt upgrade -y

# 2. Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# 4. Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Verify installation
ros2 --help
# Should show ROS 2 CLI help
```

**Test**:
```bash
# Terminal 1: Run example talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run example listener
ros2 run demo_nodes_py listener
```

You should see messages exchanged between talker and listener.

---

### Module 2: Gazebo & Unity

**Gazebo Garden**:
```bash
# 1. Add Gazebo repository
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 2. Install Gazebo Garden
sudo apt-get update
sudo apt-get install gz-garden -y

# 3. Install ROS 2 - Gazebo bridge
sudo apt install ros-humble-ros-gz -y

# 4. Test
gz sim shapes.sdf
# Should open Gazebo with 3D shapes
```

**Unity 2022.3 LTS** (optional):
- Download from https://unity.com/download
- Install Unity Hub
- Install Unity 2022.3 LTS from Hub
- Follow Module 2 lesson for ROS-TCP-Connector setup

---

### Module 3: NVIDIA Isaac Sim

**Prerequisites**:
- NVIDIA RTX GPU (2060 or newer)
- Ubuntu 22.04 LTS
- NVIDIA Driver 525+ (check with `nvidia-smi`)

**Installation**:
```bash
# 1. Install Omniverse Launcher
# Download from https://www.nvidia.com/en-us/omniverse/download/

# 2. Install Isaac Sim 2023.1+ via Launcher
# Follow on-screen instructions

# 3. Install Isaac ROS
# See Module 3 for detailed Isaac ROS setup
```

**Cloud GPU Alternative** (AWS g5.xlarge):
- See Module 5 for AWS setup instructions
- Includes NVIDIA A10G GPU, pre-configured drivers

---

### Module 4: Whisper & VLA

**Whisper (local)**:
```bash
# 1. Install Python 3.10+
sudo apt install python3 python3-pip -y

# 2. Install Whisper
pip3 install openai-whisper

# 3. Test (requires microphone or audio file)
whisper audio.mp3 --model base
```

**LLM Options**:
- **OpenAI API** (simplest): Sign up at https://platform.openai.com, get API key
- **Local Llama 3** (privacy): See Module 4 for Ollama setup instructions

---

### Module 5: Hardware Deployment

**Workstation Specs** (recommended):
- CPU: Intel i7/AMD Ryzen 7 or better
- GPU: NVIDIA RTX 3060 (12GB VRAM) or better
- RAM: 32GB DDR4
- Storage: 512GB SSD
- OS: Ubuntu 22.04 LTS

**Jetson Orin Nano** (edge compute):
- Purchase: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/
- Setup: See Module 5 for JetPack flashing and ROS 2 installation

**Sensors**:
- RealSense D435i (RGB-D camera): https://www.intelrealsense.com/depth-camera-d435i/
- IMU: MPU6050 or BNO055
- Microphone: USB or 3.5mm compatible with Ubuntu

---

## Troubleshooting

### Common Issues

**"ros2: command not found"**:
- **Cause**: ROS 2 environment not sourced
- **Fix**: Run `source /opt/ros/humble/setup.bash` or add to `~/.bashrc`

**"npm run build" fails with link errors**:
- **Cause**: Broken internal/external links
- **Fix**: Check console output, fix broken URLs or internal references

**Mermaid diagrams not rendering**:
- **Cause**: Invalid syntax or missing plugin
- **Fix**: Validate syntax at https://mermaid.live, ensure `@docusaurus/theme-mermaid` installed

**Flesch-Kincaid grade >12**:
- **Cause**: Complex sentences, jargon without definitions
- **Fix**: Simplify sentences, define acronyms, use active voice

---

## Support & Resources

- **E-Book Issues**: https://github.com/yourusername/hackathon-book-proj/issues
- **ROS 2 Answers**: https://answers.ros.org/
- **Gazebo Forum**: https://community.gazebosim.org/
- **NVIDIA Isaac Forum**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/
- **Docusaurus Docs**: https://docusaurus.io/docs

---

## Quality Checklist (Before PR)

- [ ] Docusaurus build completes (`npm run build`) with 0 errors, 0 warnings
- [ ] All code examples tested on Ubuntu 22.04
- [ ] Flesch-Kincaid readability 9-12 for all lessons
- [ ] Module has 10+ official + 5+ technical citations in sources.md
- [ ] All links resolve (internal and external)
- [ ] All Mermaid diagrams render correctly
- [ ] Screenshots/expected outputs included for code examples
- [ ] No hallucinated APIs or unverified commands
