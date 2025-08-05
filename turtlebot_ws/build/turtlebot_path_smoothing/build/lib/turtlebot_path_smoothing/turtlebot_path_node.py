#!/usr/bin/env python3
"""
TurtleBot3 Path Smoothing and Following Node
Implements cubic spline path smoothing with Stanley controller
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
import tf2_geometry_msgs

import numpy as np
import math
from typing import List, Tuple, Optional
from .cubic_spline_smoother import CubicSplinePathSmoother
from .stanley_controller import StanleyController


class TurtleBotPathSmoothingNode(Node):
    """Main ROS2 node for TurtleBot path smoothing and following"""
    
    def __init__(self):
        super().__init__('turtlebot_path_smoothing')
        
        # Initialize path smoother and controller
        self.path_smoother = CubicSplinePathSmoother(resolution=0.1)
        self.stanley_controller = StanleyController(k=0.5, k_soft=1.0, wheelbase=0.287)
        
        # Robot state
        self.current_pose = None
        self.current_velocity = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Path planning
        self.waypoints = []  # Original waypoints to visit
        self.smoothed_path = []  # Complete smoothed path through all waypoints
        self.current_waypoint_index = 1  # Index of next waypoint to reach (start from 1, not 0)
        self.visited_waypoints = []  # Track which waypoints have been visited
        self.path_following_active = False
        self.path_initialized = False
        
        # Warning flags to prevent spam
        self.warned_no_odom = False
        self.warned_no_path = False
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.25)  # Reduced tolerance for better waypoint reaching
        self.declare_parameter('max_linear_velocity', 0.4)  # Slightly reduced max speed
        self.declare_parameter('max_angular_velocity', 1.2)  # Reduced max angular velocity
        self.declare_parameter('control_frequency', 15.0)  # Increased frequency for better control
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/smoothed_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        
        # Control timer
        control_period = 1.0 / self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(control_period, self.control_callback)
        
        # Path visualization timer
        self.viz_timer = self.create_timer(1.0, self.publish_path_visualization)
        
        # Wait for odometry before initializing path
        self.path_initialized = False
        self.init_timer = self.create_timer(0.5, self.check_and_initialize_path)
        
        self.get_logger().info("TurtleBot Path Smoothing Node initialized")
    
    def check_and_initialize_path(self):
        """Check if odometry is available and initialize path"""
        if not self.path_initialized and self.current_pose is not None:
            self.initialize_demo_path()
            self.path_initialized = True
            self.init_timer.cancel()  # Stop the timer once path is initialized
    
    def initialize_demo_path(self):
        """Initialize with a demo path for testing"""
        # Get robot's current position as starting point
        if self.current_pose is None:
            self.get_logger().warn("No odometry available, using default start position")
            start_x, start_y = 0.0, 0.0
        else:
            start_x, start_y = self.robot_x, self.robot_y
            self.get_logger().info(f"Starting path from robot position: ({start_x:.2f}, {start_y:.2f})")
        
        # Option 1: S-curve path (smooth curved path)
        demo_waypoints = [
            (start_x, start_y),                    # Start at robot's current position
            # (start_x + 1.0, start_y + 0.5),       # First curve control point
            (start_x + 2.0, start_y + 1.5),       # Peak of first curve
            (start_x + 3.0, start_y + 1.8),       # Top transition point
            # (start_x + 4.0, start_y + 1.5),       # Start descent
            (start_x + 5.0, start_y),             # Middle point (straight across)
            (start_x + 6.0, start_y - 1.5),       # Start second curve
            # (start_x + 7.0, start_y - 1.8),       # Bottom transition point
            (start_x + 8.0, start_y - 1.5),       # Bottom of second curve
            # (start_x + 9.0, start_y - 0.5),       # Second curve control point
            (start_x + 10.0, start_y)             # End point
        ]
        
        # Option 2: Figure-8 path (uncomment to use)
        # demo_waypoints = [
        #     (start_x, start_y),
        #     (start_x + 1.0, start_y + 1.0),
        #     (start_x + 2.0, start_y),
        #     (start_x + 1.0, start_y - 1.0),
        #     (start_x, start_y),
        #     (start_x - 1.0, start_y + 1.0),
        #     (start_x - 2.0, start_y),
        #     (start_x - 1.0, start_y - 1.0),
        #     (start_x, start_y)
        # ]
        
        # Option 3: Circular curve path (uncomment to use)
        # import math
        # radius = 1.5
        # num_points = 8
        # demo_waypoints = []
        # for i in range(num_points + 1):
        #     theta = i * 2 * math.pi / num_points
        #     x = start_x + radius * math.cos(theta)
        #     y = start_y + radius * math.sin(theta)
        #     demo_waypoints.append((x, y))
        
        # Option 4: Heart-shaped curve (uncomment to use)
        # import math
        # demo_waypoints = []
        # scale = 0.3
        # for t in range(0, 63, 8):  # 8 points
        #     theta = t / 10.0
        #     x = start_x + scale * (16 * math.sin(theta)**3)
        #     y = start_y + scale * (13 * math.cos(theta) - 5 * math.cos(2*theta) - 2 * math.cos(3*theta) - math.cos(4*theta))
        #     demo_waypoints.append((x, y))
        # demo_waypoints.append((start_x, start_y))  # Close the loop
            
        self.get_logger().info(f"Initializing with {len(demo_waypoints)} waypoints for S-curve path")
        self.set_waypoints(demo_waypoints)
    
    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """Set new waypoints and create complete smoothed path through all of them"""
        self.waypoints = waypoints
        self.current_waypoint_index = 1  # Start targeting the second waypoint (first is start position)
        self.visited_waypoints = [False] * len(waypoints)  # Track visited status
        self.visited_waypoints[0] = True  # Mark start position as visited
        
        if len(waypoints) > 1:
            # Create one complete smoothed path through ALL waypoints
            self.smoothed_path = self.path_smoother.smooth_path(waypoints)
            self.stanley_controller.reset_path_tracking()
            self.path_following_active = True
            
            self.get_logger().info(f"Set {len(waypoints)} waypoints for complete trajectory navigation")
            self.get_logger().info(f"Generated complete smoothed path with {len(self.smoothed_path)} points")
            self.publish_path_visualization()
        else:
            self.get_logger().warn("Need at least 2 waypoints for navigation")
    
    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        self.current_pose = msg.pose.pose
        
        # Extract position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        
        # Extract linear velocity
        self.current_velocity = msg.twist.twist.linear.x
    
    def goal_callback(self, msg: PoseStamped):
        """Handle new goal from RViz"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        if self.current_pose is None:
            self.get_logger().warn("No current pose available, cannot set goal")
            return
        
        # Create path from current position to goal
        if self.current_pose is None:
            self.get_logger().warn("No current pose available, cannot set goal")
            return
        
        waypoints = [(self.robot_x, self.robot_y), (goal_x, goal_y)]
        self.set_waypoints(waypoints)
        
        self.get_logger().info(f"New goal set: ({goal_x:.2f}, {goal_y:.2f})")
    
    def control_callback(self):
        """Main control loop"""
        if self.current_pose is None:
            if not self.warned_no_odom:
                self.get_logger().warn("No odometry data received yet")
                self.warned_no_odom = True
            return
        
        if not self.smoothed_path:
            if not self.warned_no_path:
                self.get_logger().warn("No smoothed path available")
                self.warned_no_path = True
            return
        
        if not self.path_following_active:
            return
        
        # Check if we've reached any unvisited waypoints
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            if not self.visited_waypoints[i]:
                waypoint = self.waypoints[i]
                distance_to_waypoint = math.sqrt(
                    (self.robot_x - waypoint[0])**2 + 
                    (self.robot_y - waypoint[1])**2
                )
                
                waypoint_tolerance = self.get_parameter('goal_tolerance').value
                
                if distance_to_waypoint < waypoint_tolerance:
                    self.visited_waypoints[i] = True
                    self.get_logger().info(
                        f"Reached waypoint {i}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}) "
                        f"[distance: {distance_to_waypoint:.3f}m]"
                    )
                    
                    # Update current target to next unvisited waypoint
                    self.current_waypoint_index = i + 1
                    break
        
        # Check if we've reached the final waypoint (end of path)
        if len(self.smoothed_path) > 0:
            end_point = self.smoothed_path[-1]
            distance_to_end = math.sqrt(
                (self.robot_x - end_point[0])**2 + 
                (self.robot_y - end_point[1])**2
            )
            
            final_tolerance = self.get_parameter('goal_tolerance').value
            if distance_to_end < final_tolerance:
                self.get_logger().info("Reached end of trajectory! Mission complete.")
                visited_count = sum(self.visited_waypoints)
                self.get_logger().info(f"Visited {visited_count}/{len(self.waypoints)} waypoints")
                self.stop_robot()
                self.path_following_active = False
                return
        
        # Use Stanley controller for path following along the complete trajectory
        linear_vel, angular_vel = self.stanley_controller.calculate_control(
            self.robot_x, self.robot_y, self.robot_yaw, 
            self.current_velocity, self.smoothed_path
        )
        
        # Apply velocity limits
        max_linear = self.get_parameter('max_linear_velocity').value
        max_angular = self.get_parameter('max_angular_velocity').value
        
        linear_vel = max(-max_linear, min(max_linear, linear_vel))
        angular_vel = max(-max_angular, min(max_angular, angular_vel))
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def publish_path_visualization(self):
        """Publish path visualization markers"""
        if not self.smoothed_path and not self.waypoints:
            return
        
        # Use 'odom' frame for visualization (standard for TurtleBot3)
        frame_id = "odom"
        
        # Publish Path message for RViz path display
        if self.smoothed_path:
            path_msg = Path()
            path_msg.header.frame_id = frame_id
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for point in self.smoothed_path:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = point[0]
                pose_stamped.pose.position.y = point[1]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)
            
            self.path_pub.publish(path_msg)
        
        # Publish MarkerArray for detailed visualization
        marker_array = MarkerArray()
        
        # Smoothed path line
        if self.smoothed_path:
            path_marker = Marker()
            path_marker.header.frame_id = frame_id
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "smoothed_path"
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.05  # Line width
            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 1.0
            path_marker.lifetime.sec = 0  # Persist
            
            for point in self.smoothed_path:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.02
                path_marker.points.append(p)
            
            marker_array.markers.append(path_marker)
        
        # Waypoint markers (different colors for visited, current target, and future)
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color coding for waypoints based on visited status
            if hasattr(self, 'visited_waypoints') and i < len(self.visited_waypoints):
                if self.visited_waypoints[i]:
                    # Visited waypoints - green
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                elif i == self.current_waypoint_index:
                    # Current target waypoint - yellow/orange
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    # Make current target larger
                    marker.scale.x = 0.4
                    marker.scale.y = 0.4
                    marker.scale.z = 0.4
                else:
                    # Future waypoints - red
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5
            else:
                # Default red for unvisited
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
            
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)
        
        # Robot position marker (blue sphere)
        if self.current_pose is not None:
            robot_marker = Marker()
            robot_marker.header.frame_id = frame_id
            robot_marker.header.stamp = self.get_clock().now().to_msg()
            robot_marker.ns = "robot"
            robot_marker.id = 0
            robot_marker.type = Marker.SPHERE
            robot_marker.action = Marker.ADD
            robot_marker.pose.position.x = self.robot_x
            robot_marker.pose.position.y = self.robot_y
            robot_marker.pose.position.z = 0.15
            robot_marker.pose.orientation.w = 1.0
            robot_marker.scale.x = 0.3
            robot_marker.scale.y = 0.3
            robot_marker.scale.z = 0.3
            robot_marker.color.r = 0.0
            robot_marker.color.g = 0.0
            robot_marker.color.b = 1.0
            robot_marker.color.a = 1.0
            robot_marker.lifetime.sec = 0
            marker_array.markers.append(robot_marker)
        
        # Status text marker
        if self.waypoints:
            status_marker = Marker()
            status_marker.header.frame_id = frame_id
            status_marker.header.stamp = self.get_clock().now().to_msg()
            status_marker.ns = "status"
            status_marker.id = 1
            status_marker.type = Marker.TEXT_VIEW_FACING
            status_marker.action = Marker.ADD
            
            # Position text above robot
            status_marker.pose.position.x = self.robot_x if self.current_pose else 0.0
            status_marker.pose.position.y = self.robot_y + 1.0 if self.current_pose else 1.0
            status_marker.pose.position.z = 0.5
            status_marker.pose.orientation.w = 1.0
            
            status_marker.scale.z = 0.3  # Text height
            status_marker.color.r = 1.0
            status_marker.color.g = 1.0
            status_marker.color.b = 1.0
            status_marker.color.a = 1.0
            
            # Status text
            if hasattr(self, 'visited_waypoints'):
                visited_count = sum(self.visited_waypoints)
                if self.current_waypoint_index < len(self.waypoints):
                    target_wp = self.waypoints[self.current_waypoint_index]
                    status_marker.text = f"Progress: {visited_count}/{len(self.waypoints)} | Next: WP{self.current_waypoint_index} ({target_wp[0]:.1f}, {target_wp[1]:.1f})"
                else:
                    status_marker.text = f"Mission Complete! Visited: {visited_count}/{len(self.waypoints)}"
            else:
                status_marker.text = "Initializing..."
            
            status_marker.lifetime.sec = 0
            marker_array.markers.append(status_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = TurtleBotPathSmoothingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Exception in main: {e}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
