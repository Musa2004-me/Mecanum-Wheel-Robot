#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ================== IMU Splitter Node ==================
    imu_node = Node(
        package='mec',               # your package name
        executable='imu.py',         # Python script in scripts/
        name='imu_splitter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ================== Lane Detection Node ==================
    lane_node = Node(
        package='mec',
        executable='lane_detection_node.py',
        name='lane_detection_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ================== Obstacle Detection Node ==================
    obstacle_node = Node(
        package='mec',
        executable='obstacle_Detection_Node.py',
        name='obstacle_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        imu_node,
        lane_node,
        obstacle_node
    ])
