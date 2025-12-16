#!/usr/bin/env python3
"""
VSLAM Launch File for Isaac ROS

Launches:
- Isaac ROS Visual SLAM node
- RViz2 for visualization
- Optional: Static transforms

Usage:
  ros2 launch vslam_launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Isaac Sim)'
    )

    # Isaac ROS VSLAM node
    vslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_imu': False,
            'enable_rectified_pose': True,
            'rectified_images': True,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'input_left_camera_frame': 'camera_left',
            'input_right_camera_frame': 'camera_right',
        }],
        remappings=[
            ('/stereo_camera/left/image', '/camera/left/image_raw'),
            ('/stereo_camera/left/camera_info', '/camera/left/camera_info'),
            ('/stereo_camera/right/image', '/camera/right/image_raw'),
            ('/stereo_camera/right/camera_info', '/camera/right/camera_info')
        ],
        output='screen'
    )

    # RViz2 visualization
    rviz_config = os.path.join(
        get_package_share_directory('isaac_ros_visual_slam'),
        'rviz',
        'default.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static transform: base_link -> camera_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform: camera_link -> camera_left
    static_tf_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.06', '0', '0', '0', '0', 'camera_link', 'camera_left'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform: camera_link -> camera_right
    static_tf_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '-0.06', '0', '0', '0', '0', 'camera_link', 'camera_right'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        vslam_node,
        rviz_node,
        static_tf,
        static_tf_left,
        static_tf_right
    ])
