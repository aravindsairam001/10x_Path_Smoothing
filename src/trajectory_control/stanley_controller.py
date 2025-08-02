"""
Stanley Controller
Implements the Stanley lateral control algorithm for path following.
"""

import numpy as np
from typing import List, Tuple
from .differential_drive import DifferentialDrive


class StanleyController:
    """
    Stanley controller for path following.
    
    The Stanley method combines heading error and cross-track error to generate
    steering commands. It aims to reduce both the heading error and lateral
    displacement from the desired path.
    """
    
    def __init__(self, k_e: float = 0.5, k_v: float = 10.0, 
                 max_steering_angle: float = np.pi/4):
        """
        Initialize Stanley controller.
        
        Args:
            k_e: Cross-track error gain
            k_v: Velocity-dependent gain for cross-track error
            max_steering_angle: Maximum steering angle [rad]
        """
        self.k_e = k_e
        self.k_v = k_v
        self.max_steering_angle = max_steering_angle
        self.path_index = 0
    
    def find_closest_path_point(self, robot_pose: Tuple[float, float, float],
                               path: List[Tuple[float, float]]) -> Tuple[int, float, float, float]:
        """
        Find the closest point on the path and its properties.
        
        Args:
            robot_pose: Current robot pose (x, y, theta)
            path: List of (x, y) path points
            
        Returns:
            Tuple of (closest_index, cross_track_error, path_heading, distance)
        """
        robot_x, robot_y, _ = robot_pose
        
        min_distance = float('inf')
        closest_index = 0
        best_cross_track_error = 0.0
        best_path_heading = 0.0
        
        # Search for closest path segment starting from current index
        search_start = max(0, self.path_index - 5)
        search_end = min(len(path) - 1, self.path_index + 20)
        
        for i in range(search_start, search_end):
            if i >= len(path) - 1:
                continue
                
            # Current path segment
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            robot_pos = np.array([robot_x, robot_y])
            
            # Vector from p1 to p2
            segment_vec = p2 - p1
            segment_length = np.linalg.norm(segment_vec)
            
            if segment_length < 1e-6:
                continue
            
            # Project robot onto segment
            to_robot = robot_pos - p1
            t = np.dot(to_robot, segment_vec) / (segment_length**2)
            t = np.clip(t, 0, 1)  # Clamp to segment
            
            closest_point = p1 + t * segment_vec
            distance = np.linalg.norm(robot_pos - closest_point)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
                # Calculate cross-track error (positive = left of path)
                cross_product = np.cross(segment_vec, to_robot)
                best_cross_track_error = cross_product / segment_length
                
                # Calculate path heading
                best_path_heading = np.arctan2(segment_vec[1], segment_vec[0])
        
        # Update path index (don't go backwards significantly)
        self.path_index = max(self.path_index, closest_index - 2)
        
        return closest_index, best_cross_track_error, best_path_heading, min_distance
    
    def calculate_heading_error(self, robot_heading: float, 
                              path_heading: float) -> float:
        """
        Calculate heading error between robot and path.
        
        Args:
            robot_heading: Current robot heading [rad]
            path_heading: Desired path heading [rad]
            
        Returns:
            Heading error [rad], normalized to [-pi, pi]
        """
        heading_error = path_heading - robot_heading
        # Normalize to [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        return heading_error
    
    def calculate_steering_angle(self, heading_error: float,
                               cross_track_error: float,
                               velocity: float) -> float:
        """
        Calculate steering angle using Stanley control law.
        
        Args:
            heading_error: Heading error [rad]
            cross_track_error: Cross-track error [m]
            velocity: Current velocity [m/s]
            
        Returns:
            Steering angle [rad]
        """
        # Stanley control law: δ = ψ + arctan(k_e * e / (k_v + v))
        # where ψ is heading error, e is cross-track error
        
        # Avoid division by zero in velocity term
        velocity_term = self.k_v + abs(velocity)
        
        # Calculate cross-track steering component
        cross_track_steering = np.arctan(self.k_e * cross_track_error / velocity_term)
        
        # Total steering angle
        steering_angle = heading_error + cross_track_steering
        
        # Apply steering limits
        steering_angle = np.clip(steering_angle, 
                               -self.max_steering_angle,
                               self.max_steering_angle)
        
        return steering_angle
    
    def control(self, robot: DifferentialDrive,
                path: List[Tuple[float, float]],
                desired_velocity: float = 1.0) -> Tuple[float, float]:
        """
        Calculate control commands using Stanley controller.
        
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
        robot_state = robot.get_state()
        current_velocity = robot_state.v
        
        # Find closest path point and calculate errors
        closest_index, cross_track_error, path_heading, _ = \
            self.find_closest_path_point(robot_pose, path)
        
        # Calculate heading error
        heading_error = self.calculate_heading_error(robot_pose[2], path_heading)
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(
            heading_error, cross_track_error, abs(current_velocity))
        
        # Convert steering angle to angular velocity
        # For bicycle model: ω = v * tan(δ) / L
        # For differential drive approximation: ω ≈ v * δ / L (small angle)
        wheelbase = robot.params.wheelbase
        
        if abs(steering_angle) < 0.1:  # Small angle approximation
            angular_velocity = desired_velocity * steering_angle / wheelbase
        else:  # Full nonlinear model
            angular_velocity = desired_velocity * np.tan(steering_angle) / wheelbase
        
        # Apply angular velocity limits
        max_angular_velocity = robot.params.max_angular_velocity
        angular_velocity = np.clip(angular_velocity,
                                 -max_angular_velocity,
                                 max_angular_velocity)
        
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
    
    def get_control_errors(self, robot: DifferentialDrive,
                          path: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Get current control errors for monitoring.
        
        Args:
            robot: Differential drive robot
            path: List of (x, y) path points
            
        Returns:
            Tuple of (cross_track_error, heading_error)
        """
        if len(path) < 2:
            return 0.0, 0.0
        
        robot_pose = robot.get_pose()
        
        # Find closest path point and calculate errors
        _, cross_track_error, path_heading, _ = \
            self.find_closest_path_point(robot_pose, path)
        
        # Calculate heading error
        heading_error = self.calculate_heading_error(robot_pose[2], path_heading)
        
        return cross_track_error, heading_error
    
    def tune_parameters(self, k_e: float = None, k_v: float = None,
                       max_steering_angle: float = None):
        """
        Tune controller parameters.
        
        Args:
            k_e: Cross-track error gain
            k_v: Velocity-dependent gain
            max_steering_angle: Maximum steering angle [rad]
        """
        if k_e is not None:
            self.k_e = k_e
        if k_v is not None:
            self.k_v = k_v
        if max_steering_angle is not None:
            self.max_steering_angle = max_steering_angle
    
    def reset(self):
        """Reset controller state."""
        self.path_index = 0
