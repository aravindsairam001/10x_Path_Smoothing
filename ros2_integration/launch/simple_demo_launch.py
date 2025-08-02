#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
    return LaunchDescription([
        # Fake robot node (simulates TurtleBot)
        Node(
            package='path_smoothing_turtlebot',
            executable='fake_robot_node.py',
            name='fake_robot',
            output='screen'
        ),
        
        # Simple path smoothing node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='path_smoothing_turtlebot',
                    executable='simple_path_smoothing_node.py',
                    name='simple_path_smoothing',
                    output='screen'
                )
            ]
        ),
        
        # Simple trajectory controller node
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='path_smoothing_turtlebot',
                    executable='simple_trajectory_controller_node.py',
                    name='simple_trajectory_controller',
                    output='screen'
                )
            ]
        ),
        
        # Publish demo waypoints
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once', '/waypoints', 'nav_msgs/msg/Path',
                        '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \'odom\'}, poses: ['
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, '
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, '
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, '
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, '
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 0.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
                        ']}"'
                    ],
                    output='screen'
                )
            ]
        )
    ])
