#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import sys
import os

# Add the src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from obstacle_avoidance import DynamicWindowApproach
from trajectory_control import DifferentialDrive


class ObstacleAvoidanceNode(Node):
    """
    ROS2 node for obstacle avoidance using Dynamic Window Approach.
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Parameters
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('goal_cost_gain', 1.0)
        self.declare_parameter('obstacle_cost_gain', 2.0)
        self.declare_parameter('speed_cost_gain', 0.1)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goal_tolerance', 0.3)
        
        max_linear_vel = self.get_parameter('max_linear_vel').value
        max_angular_vel = self.get_parameter('max_angular_vel').value
        goal_cost_gain = self.get_parameter('goal_cost_gain').value
        obstacle_cost_gain = self.get_parameter('obstacle_cost_gain').value
        speed_cost_gain = self.get_parameter('speed_cost_gain').value
        robot_radius = self.get_parameter('robot_radius').value
        control_rate = self.get_parameter('control_rate').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # Initialize DWA
        self.dwa = DynamicWindowApproach(
            goal_cost_gain=goal_cost_gain,
            obstacle_cost_gain=obstacle_cost_gain,
            speed_cost_gain=speed_cost_gain
        )
        
        # Robot model
        self.robot = DifferentialDrive(
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            radius=robot_radius
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.trajectory_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.best_trajectory_pub = self.create_publisher(Path, '/best_trajectory', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        # State variables
        self.current_pose = None
        self.current_goal = None
        self.obstacles = []
        self.goal_reached = False
        
        self.get_logger().info('Obstacle avoidance node started')
        self.get_logger().info('Set a goal in RViz using 2D Nav Goal')
    
    def goal_callback(self, msg):
        """Receive new goal."""
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_reached = False
        self.get_logger().info(f'New goal set: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})')
    
    def odom_callback(self, msg):
        """Update current robot pose."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        
        self.current_pose = (x, y, yaw)
        
        # Update robot model
        self.robot.set_pose(x, y, yaw)
        
        # Set velocities from odometry
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v_linear = np.sqrt(vx**2 + vy**2)
        v_angular = msg.twist.twist.angular.z
        self.robot.set_velocity(v_linear, v_angular)
    
    def scan_callback(self, msg):
        """Process laser scan data to extract obstacles."""
        if self.current_pose is None:
            return
        
        x, y, yaw = self.current_pose
        obstacles = []
        
        # Convert laser scan to obstacle points
        angle = msg.angle_min
        for i, range_val in enumerate(msg.ranges):
            if msg.range_min <= range_val <= msg.range_max:
                # Convert to global coordinates
                obs_x = x + range_val * np.cos(angle + yaw)
                obs_y = y + range_val * np.sin(angle + yaw)
                obstacles.append((obs_x, obs_y))
            
            angle += msg.angle_increment
        
        self.obstacles = obstacles
    
    def control_loop(self):
        """Main control loop using DWA."""
        if (self.current_pose is None or self.current_goal is None or 
            self.goal_reached):
            self._publish_zero_velocity()
            return
        
        try:
            x, y, yaw = self.current_pose
            goal_x, goal_y = self.current_goal
            
            # Check if goal is reached
            goal_distance = np.sqrt((x - goal_x)**2 + (y - goal_y)**2)
            if goal_distance < self.goal_tolerance:
                self.goal_reached = True
                self._publish_zero_velocity()
                self.get_logger().info('Goal reached!')
                return
            
            # Run DWA
            best_v, best_w, trajectories = self.dwa.plan(
                self.robot, self.current_goal, self.obstacles
            )
            
            if best_v is None or best_w is None:
                self.get_logger().warn('DWA failed to find valid trajectory')
                self._publish_zero_velocity()
                return
            
            # Publish command
            cmd_msg = Twist()
            cmd_msg.linear.x = float(best_v)
            cmd_msg.angular.z = float(best_w)
            self.cmd_vel_pub.publish(cmd_msg)
            
            # Visualize trajectories
            self._publish_trajectory_visualization(trajectories, best_v, best_w)
            
        except Exception as e:
            self.get_logger().error(f'Error in DWA control loop: {str(e)}')
            self._publish_zero_velocity()
    
    def _publish_zero_velocity(self):
        """Stop the robot."""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
    
    def _publish_trajectory_visualization(self, trajectories, best_v, best_w):
        """Publish trajectory visualization markers."""
        if not trajectories:
            return
        
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Visualize all trajectories
        for i, (v, w, traj_points, cost) in enumerate(trajectories):
            if len(traj_points) < 2:
                continue
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dwa_trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            marker.scale.x = 0.02
            
            # Color based on whether this is the best trajectory
            if abs(v - best_v) < 0.01 and abs(w - best_w) < 0.01:
                # Best trajectory - green
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.scale.x = 0.05  # Thicker line
            else:
                # Other trajectories - red with transparency
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.3
            
            for point in traj_points:
                geometry_point = Point()
                geometry_point.x = point[0]
                geometry_point.y = point[1]
                geometry_point.z = 0.0
                marker.points.append(geometry_point)
            
            marker_array.markers.append(marker)
        
        self.trajectory_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
