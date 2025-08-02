#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


class SimpleTrajectoryControllerNode(Node):
    """
    Simplified trajectory controller node that works without external dependencies.
    """
    
    def __init__(self):
        super().__init__('simple_trajectory_controller_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/smoothed_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # State
        self.current_path = None
        self.current_pose = None
        self.current_target_idx = 0
        self.lookahead_distance = 0.5
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Simple trajectory controller node started')
    
    def path_callback(self, msg):
        """Store new path."""
        self.current_path = msg
        self.current_target_idx = 0
        self.get_logger().info(f'Received new path with {len(msg.poses)} points')
    
    def odom_callback(self, msg):
        """Store current pose."""
        self.current_pose = msg.pose.pose
    
    def control_loop(self):
        """Main control loop."""
        if self.current_path is None or self.current_pose is None:
            return
        
        if self.current_target_idx >= len(self.current_path.poses):
            # Path complete
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Find target point
        target_pose = self.current_path.poses[self.current_target_idx].pose
        
        # Calculate distance to target
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # If close to target, move to next point
        if distance < 0.2:
            self.current_target_idx += 1
            if self.current_target_idx >= len(self.current_path.poses):
                self.get_logger().info('Path completed!')
                return
            
            target_pose = self.current_path.poses[self.current_target_idx].pose
            dx = target_pose.position.x - self.current_pose.position.x
            dy = target_pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
        
        # Simple proportional controller
        cmd = Twist()
        
        # Current orientation
        current_yaw = 2 * math.atan2(self.current_pose.orientation.z, 
                                     self.current_pose.orientation.w)
        
        # Desired heading
        desired_yaw = math.atan2(dy, dx)
        
        # Angular error
        yaw_error = desired_yaw - current_yaw
        
        # Normalize angle
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Control gains
        linear_gain = 0.5
        angular_gain = 2.0
        
        # Commands
        cmd.linear.x = min(linear_gain * distance, 0.5)  # Max 0.5 m/s
        cmd.angular.z = angular_gain * yaw_error
        
        # Limit angular velocity
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleTrajectoryControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
