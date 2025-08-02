#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the scripts directory
    scripts_dir = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..',  # go up from launch to ros2_integration
        'scripts'
    )
    
    return LaunchDescription([
        # Fake robot node (simulates TurtleBot)
        Node(
            package='path_smoothing_turtlebot',
            executable='fake_robot_node.py',
            name='fake_robot',
            output='screen',
            parameters=[]
        ),
        
        # Path smoothing node (with delay to ensure fake robot starts first)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='path_smoothing_turtlebot',
                    executable='path_smoothing_node.py',
                    name='path_smoothing',
                    output='screen',
                    parameters=[
                        {'smoothing_algorithm': 'spline'},
                        {'lookahead_distance': 0.5}
                    ]
                )
            ]
        ),
        
        # Trajectory controller node (with delay)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='path_smoothing_turtlebot',
                    executable='trajectory_controller_node.py',
                    name='trajectory_controller',
                    output='screen',
                    parameters=[
                        {'controller_type': 'pure_pursuit'},
                        {'lookahead_distance': 0.5},
                        {'max_linear_velocity': 0.5},
                        {'max_angular_velocity': 1.0}
                    ]
                )
            ]
        ),
        
        # Simple waypoint publisher (publishes a basic path)
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once', '/waypoints', 'nav_msgs/msg/Path',
                        '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \'odom\'}, poses: ['
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, '
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, '
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, '
                        '{header: {frame_id: \'odom\'}, pose: {position: {x: 4.0, y: 1.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
                        ']}"'
                    ],
                    output='screen'
                )
            ]
        )
    ])
