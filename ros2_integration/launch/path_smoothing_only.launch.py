#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch only the path smoothing nodes for testing with existing simulation.
    """
    
    # Launch arguments
    smoother_type_arg = DeclareLaunchArgument(
        'smoother_type',
        default_value='spline',
        description='Path smoother type [spline, bezier, bspline]'
    )
    
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='pure_pursuit',
        description='Controller type [pure_pursuit, stanley, pid]'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch configurations
    smoother_type = LaunchConfiguration('smoother_type')
    controller_type = LaunchConfiguration('controller_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Nodes
    waypoint_publisher_node = Node(
        package='path_smoothing_turtlebot',
        executable='waypoint_publisher_node.py',
        name='waypoint_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'map',
            'default_waypoints': True
        }],
        output='screen'
    )
    
    path_smoothing_node = Node(
        package='path_smoothing_turtlebot',
        executable='path_smoothing_node.py',
        name='path_smoothing',
        parameters=[{
            'use_sim_time': use_sim_time,
            'smoother_type': smoother_type,
            'num_points': 100,
            'frame_id': 'map'
        }],
        output='screen'
    )
    
    trajectory_controller_node = Node(
        package='path_smoothing_turtlebot',
        executable='trajectory_controller_node.py',
        name='trajectory_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller_type': controller_type,
            'lookahead_distance': 1.0,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'goal_tolerance': 0.2
        }],
        output='screen'
    )
    
    # RViz configuration
    pkg_path_smoothing = FindPackageShare('path_smoothing_turtlebot')
    rviz_config_file = PathJoinSubstitution([
        pkg_path_smoothing,
        'config',
        'path_smoothing.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        smoother_type_arg,
        controller_type_arg,
        use_sim_time_arg,
        
        # Nodes
        waypoint_publisher_node,
        path_smoothing_node,  
        trajectory_controller_node,
        rviz_node,
    ])
