# Citation Guidelines (APA 7th Edition)

This guide provides formatting examples for citing sources in the Physical AI & Humanoid Robotics e-book.

## Citation Requirements

Each module **MUST** include:
- **Minimum 10 official sources** (ROS 2 documentation, NVIDIA docs, etc.)
- **Minimum 5 technical sources** (tutorials, blogs, community guides)
- All citations in **APA 7th edition format**
- Version-specific URLs when possible

## Citation Types

### 1. Official Documentation

**Format**:
```
Organization Name. (Year). Document Title. URL
```

**Examples**:

```
Open Robotics. (2023). ROS 2 Humble Concepts - Nodes. https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html

Open Robotics. (2023). ROS 2 Tutorials - Writing a Simple Publisher and Subscriber (Python). https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

NVIDIA Corporation. (2023). Isaac Sim Documentation - Getting Started. https://docs.omniverse.nvidia.com/isaacsim/latest/getting_started.html

Unity Technologies. (2023). Unity Robotics Hub Documentation. https://github.com/Unity-Technologies/Unity-Robotics-Hub

Gazebo. (2023). Gazebo Garden Documentation. https://gazebosim.org/docs/garden
```

### 2. Technical Blogs & Tutorials

**Format**:
```
Author Last Name, First Initial. (Year, Month Day). Article Title. Website Name. URL
```

**Examples**:

```
Fairchild, C. (2023, March 15). ROS 2 Tutorial for Beginners. Robotics Back-End. https://roboticsbackend.com/ros2-tutorial-for-beginners/

Articulated Robotics. (2023, June 10). Building Your First ROS 2 Package. YouTube. https://www.youtube.com/watch?v=video-id

The Construct. (2023, May 20). Mastering ROS 2 Navigation Stack. The Construct Blog. https://www.theconstructsim.com/ros2-navigation-tutorial/
```

### 3. Research Papers

**Format**:
```
Author Last Name, First Initial., & Author Last Name, First Initial. (Year). Article title. Journal Name, Volume(Issue), Page range. https://doi.org/xxx
```

**Examples**:

```
Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. Science Robotics, 7(66), eabm6074. https://doi.org/10.1126/scirobotics.abm6074

Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 3, 2149-2154. https://doi.org/10.1109/IROS.2004.1389727
```

### 4. GitHub Repositories

**Format**:
```
Author/Organization. (Year). Repository Name (Version) [Computer software]. GitHub. URL
```

**Examples**:

```
Open Robotics. (2023). ros2/rclpy (Version 3.3.13) [Computer software]. GitHub. https://github.com/ros2/rclpy

NVIDIA-Omniverse. (2023). IsaacGymEnvs [Computer software]. GitHub. https://github.com/NVIDIA-Omniverse/IsaacGymEnvs
```

### 5. Software/Tools

**Format**:
```
Organization. (Year). Software Name (Version) [Computer software]. URL
```

**Examples**:

```
Meta AI. (2023). Whisper (Version 20231117) [Computer software]. https://github.com/openai/whisper

Unitree Robotics. (2023). Unitree Go2 SDK (Version 1.0.0) [Computer software]. https://github.com/unitreerobotics/unitree_sdk2
```

## Creating a Sources Page

Each module should have a `sources.md` file following this structure:

```markdown
# Sources: Module [Number] - [Module Name]

## Official Documentation

[List all official sources here in alphabetical order by organization]

Open Robotics. (2023). ROS 2 Humble Concepts - Nodes. https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html

Open Robotics. (2023). ROS 2 Humble Concepts - Topics. https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html

## Technical Sources

[List all technical sources here in alphabetical order by author]

Fairchild, C. (2023, March 15). ROS 2 Tutorial for Beginners. Robotics Back-End. https://roboticsbackend.com/ros2-tutorial-for-beginners/

## Research Papers

[List research papers here in alphabetical order by first author]

Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. Science Robotics, 7(66), eabm6074. https://doi.org/10.1126/scirobotics.abm6074

## Software & Tools

[List software/tools here in alphabetical order by name]

Meta AI. (2023). Whisper (Version 20231117) [Computer software]. https://github.com/openai/whisper
```

## In-Text Citations

When referencing sources in lessons, use informal in-text references:

**Example 1 (Documentation)**:
```markdown
According to the official ROS 2 documentation (Open Robotics, 2023), nodes are...
```

**Example 2 (Tutorial)**:
```markdown
As explained in Fairchild's beginner tutorial (2023), the pub/sub pattern...
```

**Example 3 (Direct Link)**:
```markdown
For more details, see the [official URDF tutorial](https://docs.ros.org/en/humble/Tutorials/...).
```

## Version-Specific URLs

Always use version-specific URLs when available:

✅ **Good**: `https://docs.ros.org/en/humble/Tutorials/...`
❌ **Bad**: `https://docs.ros.org/en/latest/Tutorials/...`

✅ **Good**: `https://github.com/ros2/rclpy/tree/3.3.13/...`
❌ **Bad**: `https://github.com/ros2/rclpy/tree/main/...`

## Access Dates

For web sources without publication dates, include access date:

```
Organization Name. (n.d.). Document Title. Retrieved December 13, 2025, from URL
```

## Citation Checklist

Before submitting a module, verify:

- [ ] At least 10 official sources cited
- [ ] At least 5 technical sources cited
- [ ] All citations follow APA 7th edition format
- [ ] Version-specific URLs used when available
- [ ] URLs are HTTPS (not HTTP)
- [ ] All URLs are accessible (no broken links)
- [ ] Sources are organized by category (Official, Technical, Research)
- [ ] Sources are alphabetized within each category

## Additional Resources

- [APA 7th Edition Quick Guide](https://apastyle.apa.org/style-grammar-guidelines)
- [Purdue OWL APA Formatting](https://owl.purdue.edu/owl/research_and_citation/apa_style/apa_formatting_and_style_guide/general_format.html)
- [APA Citation Generator](https://www.scribbr.com/apa-citation-generator/)
