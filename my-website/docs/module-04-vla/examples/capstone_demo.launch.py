#!/usr/bin/env python3
"""
Capstone Demo Launch File

Launches full VLA pipeline:
- Gazebo simulation
- VSLAM
- Nav2
- Whisper node
- LLM planner
- Action orchestrator

Usage:
  export OPENAI_API_KEY="sk-..."
  ros2 launch vla_demos capstone_demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='test_arena.sdf'
    )

    # 1. Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # 2. VSLAM
    vslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('isaac_ros_visual_slam'),
                'launch', 'isaac_ros_visual_slam.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Nav2
    nav2_params = os.path.join(
        get_package_share_directory('vla_demos'),
        'config', 'nav2_params.yaml'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params
        }.items()
    )

    # 4. Whisper ROS node
    whisper_node = Node(
        package='vla_demos',
        executable='whisper_ros_node.py',
        name='whisper_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 5. LLM planner
    llm_planner = Node(
        package='vla_demos',
        executable='llm_planner.py',
        name='llm_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 6. Action orchestrator
    orchestrator = Node(
        package='vla_demos',
        executable='action_orchestrator.py',
        name='action_orchestrator',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 7. Status monitor (optional)
    status_monitor = Node(
        package='vla_demos',
        executable='status_monitor.py',
        name='status_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 8. RViz2 for visualization
    rviz_config = os.path.join(
        get_package_share_directory('vla_demos'),
        'rviz', 'capstone.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gazebo,
        vslam,
        nav2,
        whisper_node,
        llm_planner,
        orchestrator,
        status_monitor,
        rviz
    ])
