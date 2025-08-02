#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import sys
import os

# Add the src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from path_smoothing import SplineSmoother, BezierSmoother, BSplineSmoother


class PathSmoothingNode(Node):
    """
    ROS2 node for path smoothing in TurtleBot simulation.
    """
    
    def __init__(self):
        super().__init__('path_smoothing_node')
        
        # Parameters
        self.declare_parameter('smoother_type', 'spline')
        self.declare_parameter('num_points', 100)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        
        smoother_type = self.get_parameter('smoother_type').value
        self.num_points = self.get_parameter('num_points').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize smoother
        self.smoother = self._create_smoother(smoother_type)
        
        # Publishers
        self.smooth_path_pub = self.create_publisher(Path, '/smooth_path', 10)
        self.waypoints_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 10)
        self.smooth_path_markers_pub = self.create_publisher(MarkerArray, '/smooth_path_markers', 10)
        
        # Subscribers
        self.waypoints_sub = self.create_subscription(
            MarkerArray, '/waypoints', self.waypoints_callback, 10)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_path)
        
        # Current waypoints and smooth path
        self.current_waypoints = []
        self.current_smooth_path = None
        
        self.get_logger().info(f'Path smoothing node started with {smoother_type} smoother')
    
    def _create_smoother(self, smoother_type):
        """Create the appropriate smoother based on type."""
        if smoother_type == 'spline':
            return SplineSmoother()
        elif smoother_type == 'bezier':
            return BezierSmoother()
        elif smoother_type == 'bspline':
            return BSplineSmoother()
        else:
            self.get_logger().warn(f'Unknown smoother type: {smoother_type}, using spline')
            return SplineSmoother()
    
    def waypoints_callback(self, msg):
        """Process incoming waypoints and generate smooth path."""
        try:
            # Extract waypoints from markers
            waypoints = []
            for marker in msg.markers:
                if marker.type == Marker.SPHERE:
                    waypoints.append((marker.pose.position.x, marker.pose.position.y))
            
            if len(waypoints) < 2:
                self.get_logger().warn('Need at least 2 waypoints for path smoothing')
                return
            
            self.current_waypoints = waypoints
            self.get_logger().info(f'Received {len(waypoints)} waypoints')
            
            # Generate smooth path
            waypoints_array = np.array(waypoints)
            x_smooth, y_smooth = self.smoother.smooth(waypoints_array, self.num_points)
            
            # Create ROS Path message
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.frame_id
            
            for i in range(len(x_smooth)):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(x_smooth[i])
                pose.pose.position.y = float(y_smooth[i])
                pose.pose.position.z = 0.0
                
                # Calculate orientation based on path direction
                if i < len(x_smooth) - 1:
                    dx = x_smooth[i + 1] - x_smooth[i]
                    dy = y_smooth[i + 1] - y_smooth[i]
                    yaw = np.arctan2(dy, dx)
                    
                    # Convert to quaternion
                    pose.pose.orientation.z = np.sin(yaw / 2.0)
                    pose.pose.orientation.w = np.cos(yaw / 2.0)
                
                path_msg.poses.append(pose)
            
            self.current_smooth_path = path_msg
            
        except Exception as e:
            self.get_logger().error(f'Error processing waypoints: {str(e)}')
    
    def publish_path(self):
        """Publish the current smooth path and visualization markers."""
        if self.current_smooth_path is not None:
            # Publish smooth path
            self.smooth_path_pub.publish(self.current_smooth_path)
            
            # Publish waypoint markers
            if self.current_waypoints:
                self._publish_waypoint_markers()
            
            # Publish smooth path markers
            self._publish_smooth_path_markers()
    
    def _publish_waypoint_markers(self):
        """Publish visualization markers for waypoints."""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.current_waypoints):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.waypoints_pub.publish(marker_array)
    
    def _publish_smooth_path_markers(self):
        """Publish visualization markers for smooth path."""
        if self.current_smooth_path is None:
            return
        
        marker_array = MarkerArray()
        
        # Path line strip
        line_marker = Marker()
        line_marker.header = self.current_smooth_path.header
        line_marker.ns = "smooth_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        for pose in self.current_smooth_path.poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.0
            line_marker.points.append(point)
        
        marker_array.markers.append(line_marker)
        
        # Direction arrows
        arrow_skip = max(1, len(self.current_smooth_path.poses) // 10)  # Show ~10 arrows
        for i in range(0, len(self.current_smooth_path.poses), arrow_skip):
            pose = self.current_smooth_path.poses[i]
            
            arrow_marker = Marker()
            arrow_marker.header = self.current_smooth_path.header
            arrow_marker.ns = "path_directions"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            arrow_marker.pose = pose.pose
            arrow_marker.pose.position.z = 0.1  # Slightly above ground
            
            arrow_marker.scale.x = 0.3  # Length
            arrow_marker.scale.y = 0.05  # Width
            arrow_marker.scale.z = 0.05  # Height
            
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 0.7
            
            marker_array.markers.append(arrow_marker)
        
        self.smooth_path_markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = PathSmoothingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
