#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch TurtleBot3 simulation with path smoothing and trajectory control.
    """
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='World file name'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type [burger, waffle, waffle_pi]'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
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
    
    enable_obstacle_avoidance_arg = DeclareLaunchArgument(
        'enable_obstacle_avoidance',
        default_value='false',
        description='Enable obstacle avoidance using DWA'
    )
    
    # Launch configurations
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    smoother_type = LaunchConfiguration('smoother_type')
    controller_type = LaunchConfiguration('controller_type')
    enable_obstacle_avoidance = LaunchConfiguration('enable_obstacle_avoidance')
    
    # Package directories
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_path_smoothing = FindPackageShare('path_smoothing_turtlebot')
    
    # World file path
    world_file_path = PathJoinSubstitution([
        pkg_turtlebot3_gazebo,
        'worlds',
        [world, '.world']
    ])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_file_path}.items()
    )
    
    # TurtleBot3 spawn
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'spawn_turtlebot3.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.01',
            'yaw': '0.0'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Path smoothing nodes
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
    
    # Trajectory controller node (when obstacle avoidance is disabled)
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
        condition=IfCondition(['not ', enable_obstacle_avoidance]),
        output='screen'
    )
    
    # Obstacle avoidance node (when enabled)
    obstacle_avoidance_node = Node(
        package='path_smoothing_turtlebot',
        executable='obstacle_avoidance_node.py',
        name='obstacle_avoidance',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'goal_cost_gain': 1.0,
            'obstacle_cost_gain': 2.0,
            'speed_cost_gain': 0.1,
            'robot_radius': 0.2,
            'goal_tolerance': 0.3
        }],
        condition=IfCondition(enable_obstacle_avoidance),
        output='screen'
    )
    
    # RViz
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
        world_arg,
        model_arg,
        use_sim_time_arg,
        smoother_type_arg,
        controller_type_arg,
        enable_obstacle_avoidance_arg,
        
        # Launch files
        gazebo_launch,
        spawn_turtlebot_cmd,
        robot_state_publisher_cmd,
        
        # Our nodes
        waypoint_publisher_node,
        path_smoothing_node,
        trajectory_controller_node,
        obstacle_avoidance_node,
        rviz_node,
    ])
