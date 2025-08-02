#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import sys
import os

# Add the src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from trajectory_control import PurePursuitController, StanleyController, PIDController
from trajectory_control import DifferentialDrive


class TrajectoryControllerNode(Node):
    """
    ROS2 node for trajectory control of TurtleBot in simulation.
    """
    
    def __init__(self):
        super().__init__('trajectory_controller_node')
        
        # Parameters
        self.declare_parameter('controller_type', 'pure_pursuit')
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        
        controller_type = self.get_parameter('controller_type').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        control_rate = self.get_parameter('control_rate').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Initialize controller
        self.controller = self._create_controller(controller_type)
        
        # Robot model for simulation
        self.robot = DifferentialDrive(
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel
        )
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/smooth_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        # State variables
        self.current_path = None
        self.current_pose = None
        self.goal_reached = False
        self.path_index = 0
        
        self.get_logger().info(f'Trajectory controller node started with {controller_type} controller')
    
    def _create_controller(self, controller_type):
        """Create the appropriate controller based on type."""
        if controller_type == 'pure_pursuit':
            return PurePursuitController(lookahead_distance=self.lookahead_distance)
        elif controller_type == 'stanley':
            return StanleyController()
        elif controller_type == 'pid':
            return PIDController()
        else:
            self.get_logger().warn(f'Unknown controller type: {controller_type}, using pure_pursuit')
            return PurePursuitController(lookahead_distance=self.lookahead_distance)
    
    def path_callback(self, msg):
        """Receive new path to follow."""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.current_path = msg
        self.path_index = 0
        self.goal_reached = False
        
        self.get_logger().info(f'Received new path with {len(msg.poses)} poses')
    
    def odom_callback(self, msg):
        """Update current robot pose from odometry."""
        self.current_pose = msg.pose.pose
        
        # Publish current pose for visualization
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.map_frame
        pose_stamped.pose = self.current_pose
        self.current_pose_pub.publish(pose_stamped)
    
    def control_loop(self):
        """Main control loop."""
        if self.current_path is None or self.current_pose is None or self.goal_reached:
            # No path or goal reached, stop the robot
            self._publish_zero_velocity()
            return
        
        try:
            # Convert path to numpy array for controller
            path_points = []
            for pose in self.current_path.poses:
                path_points.append([
                    pose.pose.position.x,
                    pose.pose.position.y
                ])
            path_array = np.array(path_points)
            
            # Get current robot state
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            
            # Extract yaw from quaternion
            orientation = self.current_pose.orientation
            yaw = np.arctan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            )
            
            # Update robot model with current state
            self.robot.set_pose(robot_x, robot_y, yaw)
            
            # Check if goal is reached
            goal_pose = self.current_path.poses[-1]
            goal_distance = np.sqrt(
                (robot_x - goal_pose.pose.position.x)**2 + 
                (robot_y - goal_pose.pose.position.y)**2
            )
            
            if goal_distance < self.goal_tolerance:
                self.goal_reached = True
                self._publish_zero_velocity()
                self.get_logger().info('Goal reached!')
                return
            
            # Calculate control commands
            if isinstance(self.controller, PurePursuitController):
                linear_vel, angular_vel = self.controller.control(self.robot, path_array)
            elif isinstance(self.controller, StanleyController):
                linear_vel, angular_vel = self.controller.control(self.robot, path_array)
            else:  # PID controller
                # For PID, we need to find the closest point on path
                distances = np.sqrt(np.sum((path_array - np.array([robot_x, robot_y]))**2, axis=1))
                closest_idx = np.argmin(distances)
                target_point = path_array[min(closest_idx + 1, len(path_array) - 1)]
                
                linear_vel, angular_vel = self.controller.control(
                    self.robot,
                    target_point[0],
                    target_point[1]
                )
            
            # Apply velocity limits
            linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
            angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
            
            # Publish command
            cmd_msg = Twist()
            cmd_msg.linear.x = float(linear_vel)
            cmd_msg.angular.z = float(angular_vel)
            self.cmd_vel_pub.publish(cmd_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            self._publish_zero_velocity()
    
    def _publish_zero_velocity(self):
        """Publish zero velocity command to stop the robot."""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
