#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from interactive_markers import InteractiveMarkerServer
import numpy as np


class WaypointPublisherNode(Node):
    """
    ROS2 node for publishing waypoints through interactive markers in RViz.
    """
    
    def __init__(self):
        super().__init__('waypoint_publisher_node')
        
        # Parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_scale', 0.3)
        self.declare_parameter('default_waypoints', True)
        
        self.frame_id = self.get_parameter('frame_id').value
        self.marker_scale = self.get_parameter('marker_scale').value
        use_default = self.get_parameter('default_waypoints').value
        
        # Interactive marker server
        self.server = InteractiveMarkerServer(self, 'waypoint_markers')
        
        # Publishers
        self.waypoints_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        
        # Waypoints storage
        self.waypoints = []
        self.waypoint_counter = 0
        
        # Timer for publishing waypoints
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        
        if use_default:
            self._create_default_waypoints()
        
        self.get_logger().info('Waypoint publisher node started')
        self.get_logger().info('Use RViz Interactive Markers to add/move waypoints')
    
    def _create_default_waypoints(self):
        """Create a set of default waypoints for demonstration."""
        default_points = [
            (0.0, 0.0),
            (3.0, 1.0),
            (5.0, 3.0),
            (7.0, 2.0),
            (10.0, 0.0),
            (8.0, -2.0),
            (5.0, -1.0),
            (2.0, -2.0)
        ]
        
        for x, y in default_points:
            self._add_waypoint(x, y)
    
    def _add_waypoint(self, x, y):
        """Add a new waypoint at the specified position."""
        waypoint_id = f"waypoint_{self.waypoint_counter}"
        self.waypoint_counter += 1
        
        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = waypoint_id
        int_marker.description = f"Waypoint {len(self.waypoints) + 1}"
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.w = 1.0
        
        # Create sphere marker
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = self.marker_scale
        sphere_marker.scale.y = self.marker_scale
        sphere_marker.scale.z = self.marker_scale
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 1.0
        
        # Create interactive marker control
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_marker)
        int_marker.controls.append(sphere_control)
        
        # Create movement controls
        move_control = InteractiveMarkerControl()
        move_control.name = "move_xy"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.orientation.w = 1.0
        move_control.orientation.x = 0.0
        move_control.orientation.y = 1.0
        move_control.orientation.z = 0.0
        int_marker.controls.append(move_control)
        
        # Add to server with callback
        self.server.insert(int_marker, self._waypoint_feedback)
        
        # Store waypoint info
        waypoint_info = {
            'id': waypoint_id,
            'x': x,
            'y': y
        }
        self.waypoints.append(waypoint_info)
        
        self.server.applyChanges()
        self.get_logger().info(f'Added waypoint at ({x:.2f}, {y:.2f})')
    
    def _waypoint_feedback(self, feedback):
        """Handle feedback from interactive markers."""
        waypoint_id = feedback.marker_name
        
        # Find and update waypoint
        for waypoint in self.waypoints:
            if waypoint['id'] == waypoint_id:
                waypoint['x'] = feedback.pose.position.x
                waypoint['y'] = feedback.pose.position.y
                break
        
        self.get_logger().debug(f'Updated {waypoint_id} position')
    
    def publish_waypoints(self):
        """Publish current waypoints as MarkerArray."""
        if not self.waypoints:
            return
        
        marker_array = MarkerArray()
        
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = waypoint['x']
            marker.pose.position.y = waypoint['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = self.marker_scale
            marker.scale.y = self.marker_scale
            marker.scale.z = self.marker_scale
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        # Add line connecting waypoints
        if len(self.waypoints) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "waypoint_connections"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            line_marker.scale.x = 0.02  # Line width
            line_marker.color.r = 0.5
            line_marker.color.g = 0.5
            line_marker.color.b = 0.5
            line_marker.color.a = 0.8
            
            for waypoint in self.waypoints:
                point = Point()
                point.x = waypoint['x']
                point.y = waypoint['y']
                point.z = 0.0
                line_marker.points.append(point)
            
            marker_array.markers.append(line_marker)
        
        self.waypoints_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
