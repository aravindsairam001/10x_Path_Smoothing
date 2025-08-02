"""
Pure Pursuit Controller
Implements the pure pursuit path following algorithm for differential drive robots.
"""

import numpy as np
from typing import List, Tuple, Optional
from .differential_drive import DifferentialDrive, RobotState


class PurePursuitController:
    """
    Pure pursuit controller for path following.
    
    The pure pursuit algorithm calculates the angular velocity needed to follow
    a path by looking ahead to a target point on the path and steering towards it.
    """
    
    def __init__(self, lookahead_distance: float = 2.0, 
                 k_dd: float = 0.1,
                 max_angular_velocity: float = 1.0):
        """
        Initialize pure pursuit controller.
        
        Args:
            lookahead_distance: Fixed lookahead distance [m]
            k_dd: Dynamic lookahead gain (scales with velocity)
            max_angular_velocity: Maximum angular velocity [rad/s]
        """
        self.lookahead_distance = lookahead_distance
        self.k_dd = k_dd  # Dynamic lookahead gain
        self.max_angular_velocity = max_angular_velocity
        self.path_index = 0  # Current closest point index on path
    
    def find_lookahead_point(self, robot_pose: Tuple[float, float, float],
                           path: List[Tuple[float, float]],
                           current_velocity: float = 0.0) -> Tuple[float, float, int]:
        """
        Find the lookahead point on the path.
        
        Args:
            robot_pose: Current robot pose (x, y, theta)
            path: List of (x, y) path points
            current_velocity: Current robot velocity for dynamic lookahead
            
        Returns:
            Tuple of (lookahead_x, lookahead_y, path_index)
        """
        robot_x, robot_y, _ = robot_pose
        
        # Dynamic lookahead distance
        dynamic_lookahead = self.lookahead_distance + self.k_dd * abs(current_velocity)
        
        # Find closest point on path
        min_distance = float('inf')
        closest_index = 0
        
        for i, (px, py) in enumerate(path):
            distance = np.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # Update path index (don't go backwards)
        self.path_index = max(self.path_index, closest_index)
        
        # Find lookahead point starting from closest point
        for i in range(self.path_index, len(path)):
            px, py = path[i]
            distance = np.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            
            if distance >= dynamic_lookahead:
                return px, py, i
        
        # If no point found at lookahead distance, return last point
        if len(path) > 0:
            return path[-1][0], path[-1][1], len(path) - 1
        else:
            return robot_x, robot_y, 0
    
    def calculate_steering_angle(self, robot_pose: Tuple[float, float, float],
                               lookahead_point: Tuple[float, float]) -> float:
        """
        Calculate steering angle to reach lookahead point.
        
        Args:
            robot_pose: Current robot pose (x, y, theta)
            lookahead_point: Target lookahead point (x, y)
            
        Returns:
            Steering angle [rad]
        """
        robot_x, robot_y, robot_theta = robot_pose
        target_x, target_y = lookahead_point
        
        # Calculate angle to target
        target_angle = np.arctan2(target_y - robot_y, target_x - robot_x)
        
        # Calculate steering angle (difference between target and current heading)
        steering_angle = target_angle - robot_theta
        
        # Normalize to [-pi, pi]
        steering_angle = np.arctan2(np.sin(steering_angle), np.cos(steering_angle))
        
        return steering_angle
    
    def calculate_curvature(self, robot_pose: Tuple[float, float, float],
                          lookahead_point: Tuple[float, float]) -> float:
        """
        Calculate path curvature for pure pursuit control.
        
        Args:
            robot_pose: Current robot pose (x, y, theta)
            lookahead_point: Target lookahead point (x, y)
            
        Returns:
            Path curvature [1/m]
        """
        robot_x, robot_y, robot_theta = robot_pose
        target_x, target_y = lookahead_point
        
        # Distance to lookahead point
        L = np.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
        
        if L < 1e-6:  # Avoid division by zero
            return 0.0
        
        # Calculate lateral error (cross-track error)
        # Transform target point to robot frame
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # Rotate to robot frame
        cross_track_error = -dx * np.sin(robot_theta) + dy * np.cos(robot_theta)
        
        # Calculate curvature using pure pursuit formula
        curvature = 2 * cross_track_error / (L**2)
        
        return curvature
    
    def control(self, robot: DifferentialDrive, 
                path: List[Tuple[float, float]],
                desired_velocity: float = 1.0) -> Tuple[float, float]:
        """
        Calculate control commands for pure pursuit path following.
        
        Args:
            robot: Differential drive robot
            path: List of (x, y) path points
            desired_velocity: Desired forward velocity [m/s]
            
        Returns:
            Tuple of (linear_velocity, angular_velocity) commands
        """
        if len(path) < 2:
            return 0.0, 0.0
        
        robot_pose = robot.get_pose()
        current_velocity = robot.get_state().v
        
        # Find lookahead point
        lookahead_x, lookahead_y, _ = self.find_lookahead_point(
            robot_pose, path, current_velocity)
        
        # Calculate curvature
        curvature = self.calculate_curvature(robot_pose, (lookahead_x, lookahead_y))
        
        # Calculate angular velocity from curvature
        angular_velocity = curvature * desired_velocity
        
        # Apply angular velocity limits
        angular_velocity = np.clip(angular_velocity, 
                                 -self.max_angular_velocity,
                                 self.max_angular_velocity)
        
        # Limit velocities based on robot constraints
        linear_velocity, angular_velocity = robot.limit_velocities(
            desired_velocity, angular_velocity)
        
        return linear_velocity, angular_velocity
    
    def is_goal_reached(self, robot_pose: Tuple[float, float, float],
                       goal: Tuple[float, float],
                       tolerance: float = 0.2) -> bool:
        """
        Check if robot has reached the goal.
        
        Args:
            robot_pose: Current robot pose (x, y, theta)
            goal: Goal position (x, y)
            tolerance: Distance tolerance [m]
            
        Returns:
            True if goal is reached
        """
        robot_x, robot_y, _ = robot_pose
        goal_x, goal_y = goal
        
        distance = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        return distance <= tolerance
    
    def get_cross_track_error(self, robot_pose: Tuple[float, float, float],
                             path: List[Tuple[float, float]]) -> float:
        """
        Calculate cross-track error (lateral deviation from path).
        
        Args:
            robot_pose: Current robot pose (x, y, theta)
            path: List of (x, y) path points
            
        Returns:
            Cross-track error [m] (positive = left of path)
        """
        if len(path) < 2:
            return 0.0
        
        robot_x, robot_y, _ = robot_pose
        
        # Find closest path segment
        min_distance = float('inf')
        closest_segment_start = 0
        
        for i in range(len(path) - 1):
            # Project robot position onto path segment
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            robot_pos = np.array([robot_x, robot_y])
            
            # Vector from p1 to p2
            segment_vec = p2 - p1
            segment_length = np.linalg.norm(segment_vec)
            
            if segment_length < 1e-6:
                continue
            
            # Project robot onto segment
            t = np.dot(robot_pos - p1, segment_vec) / (segment_length**2)
            t = np.clip(t, 0, 1)  # Clamp to segment
            
            closest_point = p1 + t * segment_vec
            distance = np.linalg.norm(robot_pos - closest_point)
            
            if distance < min_distance:
                min_distance = distance
                closest_segment_start = i
        
        # Calculate cross-track error for closest segment
        if closest_segment_start < len(path) - 1:
            p1 = np.array(path[closest_segment_start])
            p2 = np.array(path[closest_segment_start + 1])
            robot_pos = np.array([robot_x, robot_y])
            
            # Vector from p1 to p2
            segment_vec = p2 - p1
            segment_length = np.linalg.norm(segment_vec)
            
            if segment_length > 1e-6:
                # Calculate cross product to determine side
                to_robot = robot_pos - p1
                cross_product = np.cross(segment_vec, to_robot)
                return cross_product / segment_length
        
        return 0.0
    
    def reset(self):
        """Reset controller state."""
        self.path_index = 0
