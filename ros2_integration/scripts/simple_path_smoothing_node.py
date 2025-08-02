#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class SimplePathSmoothingNode(Node):
    """
    Simplified path smoothing node that works without external dependencies.
    """
    
    def __init__(self):
        super().__init__('simple_path_smoothing_node')
        
        # Publishers
        self.smoothed_path_pub = self.create_publisher(Path, '/smoothed_path', 10)
        
        # Subscribers
        self.waypoints_sub = self.create_subscription(
            Path, '/waypoints', self.waypoints_callback, 10)
        
        self.get_logger().info('Simple path smoothing node started')
    
    def waypoints_callback(self, msg):
        """Process waypoints and publish smoothed path."""
        self.get_logger().info(f'Received {len(msg.poses)} waypoints')
        
        if len(msg.poses) < 2:
            return
        
        # Simple smoothing - just interpolate between points
        smoothed_path = Path()
        smoothed_path.header = msg.header
        
        for i in range(len(msg.poses) - 1):
            start_pose = msg.poses[i]
            end_pose = msg.poses[i + 1]
            
            # Add start point
            smoothed_path.poses.append(start_pose)
            
            # Add interpolated points
            num_interp = 5
            for j in range(1, num_interp):
                t = j / num_interp
                
                interp_pose = PoseStamped()
                interp_pose.header = start_pose.header
                
                # Linear interpolation
                interp_pose.pose.position.x = (1 - t) * start_pose.pose.position.x + t * end_pose.pose.position.x
                interp_pose.pose.position.y = (1 - t) * start_pose.pose.position.y + t * end_pose.pose.position.y
                interp_pose.pose.position.z = 0.0
                
                # Simple orientation
                interp_pose.pose.orientation.w = 1.0
                
                smoothed_path.poses.append(interp_pose)
        
        # Add final point
        smoothed_path.poses.append(msg.poses[-1])
        
        self.smoothed_path_pub.publish(smoothed_path)
        self.get_logger().info(f'Published smoothed path with {len(smoothed_path.poses)} points')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimplePathSmoothingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
