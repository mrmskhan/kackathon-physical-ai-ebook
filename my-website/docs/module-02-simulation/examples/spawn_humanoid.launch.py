#!/usr/bin/env python3
"""
Spawn Humanoid Robot in Gazebo Garden

This launch file:
1. Starts Gazebo Garden with a world file
2. Spawns a humanoid robot from URDF
3. Bridges ROS 2 topics to Gazebo

Usage:
  ros2 launch spawn_humanoid.launch.py
  
Requirements:
  - ros-humble-ros-gz
  - Humanoid URDF from Module 1 (humanoid_basic.urdf)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for spawning humanoid in Gazebo."""
    
    # Launch arguments
    world_file = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='simple_world.sdf',
        description='Path to world file'
    )
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid',
        description='Name of the robot in simulation'
    )
    
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position to spawn robot'
    )
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position to spawn robot'
    )
    
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.5',
        description='Z position to spawn robot (height above ground)'
    )
    
    # Start Gazebo Garden
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot from URDF
    # Note: Update the path to your humanoid_basic.urdf location
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-file', 'path/to/humanoid_basic.urdf',  # Update this path
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-allow_renaming', 'false'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for /clock topic
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for robot pose
    bridge_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/model/', robot_name, '/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
        ],
        output='screen'
    )
    
    # Robot state publisher (publishes TF transforms from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': 'path/to/humanoid_basic.urdf'  # Update this
        }],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    
    # Add nodes
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(bridge_clock)
    ld.add_action(bridge_pose)
    ld.add_action(robot_state_publisher)
    
    return ld
