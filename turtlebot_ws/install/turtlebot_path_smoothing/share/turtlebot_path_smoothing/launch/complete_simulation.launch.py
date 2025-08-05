#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Set the TurtleBot3 model environment variable
    turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    
    # Get the launch directory
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Include the TurtleBot3 empty world Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo, 'launch'),
            '/empty_world.launch.py'
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )
    
    # Path smoothing node
    path_smoothing_node = Node(
        package='turtlebot_path_smoothing',
        executable='turtlebot_path_node',
        name='turtlebot_path_smoothing',
        output='screen',
        parameters=[{
            'goal_tolerance': 0.25,        # Tighter tolerance for better waypoint reaching
            'max_linear_velocity': 0.4,    # Slightly reduced for better control
            'max_angular_velocity': 1.2,   # Reduced for smoother turns
            'control_frequency': 15.0      # Higher frequency for better control
        }]
    )
    
    # RViz2 with custom config
    rviz_config_file = os.path.join(
        get_package_share_directory('turtlebot_path_smoothing'),
        'rviz',
        'turtlebot_path_smoothing.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Delay the path smoothing node to ensure Gazebo is fully loaded
    delayed_path_node = TimerAction(
        period=5.0,
        actions=[path_smoothing_node]
    )
    
    # Delay RViz to ensure topics are available
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        # Set environment variable for TurtleBot3 model
        turtlebot3_model,
        
        gazebo_launch,
        delayed_rviz,
        delayed_path_node,
    ])
