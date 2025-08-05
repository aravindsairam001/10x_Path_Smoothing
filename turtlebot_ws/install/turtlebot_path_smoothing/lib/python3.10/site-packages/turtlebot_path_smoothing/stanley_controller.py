#!/usr/bin/env python3
"""
Stanley Controller for TurtleBot3 Path Following
"""

import numpy as np
import math
from typing import List, Tuple, Optional


class StanleyController:
    """
    Stanley controller for path following on differential drive robots like TurtleBot3
    
    The Stanley method combines heading error and cross-track error to generate
    steering commands for smooth path following.
    """
    
    def __init__(self, k: float = 0.5, k_soft: float = 1.0, wheelbase: float = 0.287):
        """
        Initialize Stanley controller
        
        Args:
            k: Cross-track error gain
            k_soft: Velocity softening gain  
            wheelbase: Wheelbase length of the robot (m) - for TurtleBot3 burger: 0.287m
        """
        self.k = k
        self.k_soft = k_soft
        self.wheelbase = wheelbase
        self.current_segment_idx = 0
        self.max_angular_velocity = 1.5  # rad/s
        
    def reset_path_tracking(self):
        """Reset the controller state for a new path"""
        self.current_segment_idx = 0
        
    def find_nearest_point_on_path(self, robot_x: float, robot_y: float, 
                                  path_points: List[Tuple[float, float]]) -> Tuple[int, float, float, float]:
        """
        Find the nearest point on the path to the robot's current position
        Uses progressive search to avoid jumping backwards on the path
        
        Args:
            robot_x, robot_y: Current robot position
            path_points: List of (x, y) coordinates for the path
            
        Returns:
            (nearest_idx, cross_track_error, path_heading, distance_to_point)
        """
        if not path_points:
            return -1, 0.0, 0.0, float('inf')
        
        # Ensure current segment index is valid
        if self.current_segment_idx >= len(path_points) - 1:
            self.current_segment_idx = len(path_points) - 2
        if self.current_segment_idx < 0:
            self.current_segment_idx = 0
        
        min_distance = float('inf')
        nearest_idx = self.current_segment_idx
        cross_track_error = 0.0
        path_heading = 0.0
        
        # Search forward from current segment for better continuity
        search_start = max(0, self.current_segment_idx - 2)  # Allow small lookback
        search_end = min(len(path_points) - 1, self.current_segment_idx + 10)  # Lookahead window
        
        for i in range(search_start, search_end):
            # Get current path segment
            p1 = np.array(path_points[i])
            p2 = np.array(path_points[i + 1]) if i + 1 < len(path_points) else p1
            
            # Vector from p1 to p2 (path segment)
            path_vec = p2 - p1
            path_length = np.linalg.norm(path_vec)
            
            if path_length < 1e-6:  # Skip tiny segments
                continue
            
            # Vector from p1 to robot
            robot_pos = np.array([robot_x, robot_y])
            robot_vec = robot_pos - p1
            
            # Project robot position onto the path segment
            t = np.dot(robot_vec, path_vec) / (path_length**2)
            t = max(0.0, min(1.0, t))  # Clamp to segment
            
            # Calculate closest point on segment
            closest_point = p1 + t * path_vec
            
            # Distance to closest point
            distance = np.linalg.norm(robot_pos - closest_point)
            
            # Update if this is the closest segment
            if distance < min_distance:
                min_distance = distance
                nearest_idx = i
                
                # Calculate cross-track error (signed distance)
                # Positive: robot is to the left of path, Negative: robot is to the right
                cross_product = np.cross(path_vec, robot_vec)
                cross_track_error = cross_product / path_length
                
                # Calculate path heading
                path_heading = math.atan2(path_vec[1], path_vec[0])
        
        # Update current segment index for next iteration (progress along path)
        if nearest_idx > self.current_segment_idx:
            self.current_segment_idx = nearest_idx
        
        return nearest_idx, cross_track_error, path_heading, min_distance
    
    def calculate_control(self, robot_x: float, robot_y: float, robot_yaw: float,
                         robot_velocity: float, path_points: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calculate control commands using Stanley control law
        
        Args:
            robot_x, robot_y: Current robot position
            robot_yaw: Current robot heading (radians)
            robot_velocity: Current robot velocity (m/s)
            path_points: Path points as [(x1,y1), (x2,y2), ...]
            
        Returns:
            (linear_velocity, angular_velocity)
        """
        if not path_points or len(path_points) < 2:
            return 0.1, 0.0  # Default slow forward movement
        
        # Check if we're close to the goal
        goal_point = path_points[-1]
        distance_to_goal = math.sqrt((robot_x - goal_point[0])**2 + (robot_y - goal_point[1])**2)
        
        # If very close to goal, use simple goal-seeking behavior
        if distance_to_goal < 0.5:
            goal_heading = math.atan2(goal_point[1] - robot_y, goal_point[0] - robot_x)
            heading_error = goal_heading - robot_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            
            # Slow down as we approach the goal
            linear_velocity = max(0.1, 0.3 * (distance_to_goal / 0.5))
            angular_velocity = heading_error * 1.5  # Higher gain for final approach
            
            # Limit angular velocity
            angular_velocity = max(-self.max_angular_velocity, 
                                 min(self.max_angular_velocity, angular_velocity))
            
            return linear_velocity, angular_velocity
        
        # Find nearest point on path
        nearest_idx, cte, path_heading, distance = self.find_nearest_point_on_path(
            robot_x, robot_y, path_points
        )
        
        if nearest_idx == -1:
            return 0.1, 0.0  # Default movement if no valid point found
        
        # Look ahead on the path for better trajectory following
        lookahead_distance = 0.5  # meters
        lookahead_idx = nearest_idx
        
        # Find lookahead point
        accumulated_distance = 0.0
        for i in range(nearest_idx, len(path_points) - 1):
            p1 = np.array(path_points[i])
            p2 = np.array(path_points[i + 1])
            segment_length = np.linalg.norm(p2 - p1)
            
            if accumulated_distance + segment_length >= lookahead_distance:
                # Interpolate to exact lookahead distance
                remaining_distance = lookahead_distance - accumulated_distance
                t = remaining_distance / segment_length if segment_length > 0 else 0
                lookahead_point = p1 + t * (p2 - p1)
                break
            
            accumulated_distance += segment_length
            lookahead_idx = i + 1
        else:
            # Use the last point if we can't find exact lookahead distance
            lookahead_point = np.array(path_points[-1])
        
        # Calculate heading to lookahead point
        lookahead_heading = math.atan2(
            lookahead_point[1] - robot_y, 
            lookahead_point[0] - robot_x
        )
        
        # Calculate heading error using lookahead point
        heading_error = lookahead_heading - robot_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Base linear velocity
        base_velocity = 0.25  # m/s - reduced for better control
        
        # Ensure minimum velocity to avoid motor deadband issues
        velocity_for_control = max(self.k_soft, abs(robot_velocity))
        if velocity_for_control < 0.1:
            velocity_for_control = 0.1
        
        # Cross-track error term (reduce gain for smoother control)
        cte_term = math.atan2(self.k * 0.5 * cte, velocity_for_control)
        
        # Stanley steering angle: heading error + cross-track error correction
        steering_angle = heading_error + cte_term
        
        # Convert steering angle to angular velocity
        if abs(steering_angle) < 0.2:  # Small angle approximation
            angular_velocity = base_velocity * math.tan(steering_angle) / self.wheelbase
        else:
            # For larger steering angles, use proportional control
            angular_velocity = steering_angle * 1.2  # Increased gain for responsiveness
        
        # Limit angular velocity
        angular_velocity = max(-self.max_angular_velocity, 
                             min(self.max_angular_velocity, angular_velocity))
        
        # Adjust linear velocity based on angular velocity (slow down for turns)
        if abs(angular_velocity) > 0.5:
            linear_velocity = base_velocity * 0.6  # Slow down for sharp turns
        elif abs(angular_velocity) > 0.3:
            linear_velocity = base_velocity * 0.8  # Moderate slowdown
        else:
            linear_velocity = base_velocity  # Normal speed for straight sections
        
        # Ensure minimum velocity
        linear_velocity = max(0.1, linear_velocity)
        
        return linear_velocity, angular_velocity
    
    def is_goal_reached(self, robot_x: float, robot_y: float, 
                       path_points: List[Tuple[float, float]], tolerance: float = 0.2) -> bool:
        """
        Check if the robot has reached the end of the path
        
        Args:
            robot_x, robot_y: Current robot position
            path_points: Path points
            tolerance: Distance tolerance for goal reaching
            
        Returns:
            True if goal is reached
        """
        if not path_points:
            return False
        
        goal_point = path_points[-1]
        distance = math.sqrt((robot_x - goal_point[0])**2 + (robot_y - goal_point[1])**2)
        
        return distance < tolerance
