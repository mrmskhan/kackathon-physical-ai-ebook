---
id: capstone-index
title: "Capstone Project: Full VLA System"
sidebar_position: 1
---

# Capstone Project: Full VLA System

Build a complete voice-controlled humanoid robot that integrates all modules.

## Project Goal

Create an end-to-end Vision-Language-Action (VLA) system where:
1. User speaks: "Robot, go to the kitchen and bring me the cup"
2. Whisper transcribes speech
3. LLM decomposes into action sequence
4. ROS 2 executes navigation and manipulation
5. Robot provides voice feedback on completion

## Prerequisites

**Required Modules:**
- Module 1: ROS 2 Fundamentals
- Module 2: Gazebo Simulation
- Module 3: AI-Powered Perception & Navigation
- Module 4: Vision-Language-Action Integration

**Optional:**
- Module 5: Hardware Deployment (for real robot deployment)

## Expected Outcome

A working system demonstrating:
- Voice command recognition (>90% accuracy)
- Task decomposition with LLM
- Autonomous navigation with Nav2
- Error handling and recovery
- Voice feedback to user

## Time Estimate

12-15 hours for full integration and testing

## Hardware Requirements

**Simulation (Free):**
- Development workstation from Module 1
- Runs entirely in Gazebo

**Physical Deployment (Optional):**
- + Jetson Orin ($500)
- + RealSense camera ($330)
- + Robot platform ($1,600-$15,000)

## Project Structure

1. [Architecture](./architecture.md) - System design and data flow
2. [Implementation](./implementation.md) - Step-by-step integration guide
3. [Troubleshooting](./troubleshooting.md) - Common issues and solutions

## Success Criteria

- [ ] Voice commands transcribed accurately
- [ ] LLM generates valid action sequences
- [ ] Robot navigates to target locations
- [ ] Error recovery works gracefully
- [ ] End-to-end latency less than 5 seconds

## Next Steps

Start with [Architecture](./architecture.md) to understand the system design.
