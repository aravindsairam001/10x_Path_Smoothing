"""
PID Controller
Implements PID-based trajectory tracking for differential drive robots.
"""

import numpy as np
from typing import List, Tuple, Optional
from .differential_drive import DifferentialDrive


class PIDController:
    """
    PID controller for trajectory tracking.
    
    Uses separate PID controllers for lateral and heading control
    to track a reference trajectory.
    """
    
    def __init__(self, kp_lateral: float = 1.0, ki_lateral: float = 0.1, 
                 kd_lateral: float = 0.05,
                 kp_heading: float = 2.0, ki_heading: float = 0.1,
                 kd_heading: float = 0.1,
                 max_output: float = 2.0,
                 integral_windup_limit: float = 1.0):
        """
        Initialize PID controller.
        
        Args:
            kp_lateral: Proportional gain for lateral control
            ki_lateral: Integral gain for lateral control
            kd_lateral: Derivative gain for lateral control
            kp_heading: Proportional gain for heading control
            ki_heading: Integral gain for heading control
            kd_heading: Derivative gain for heading control
            max_output: Maximum controller output
            integral_windup_limit: Limit for integral windup
        """
        # Lateral PID parameters
        self.kp_lateral = kp_lateral
        self.ki_lateral = ki_lateral
        self.kd_lateral = kd_lateral
        
        # Heading PID parameters
        self.kp_heading = kp_heading
        self.ki_heading = ki_heading
        self.kd_heading = kd_heading
        
        # Control limits
        self.max_output = max_output
        self.integral_windup_limit = integral_windup_limit
        
        # Internal state
        self.lateral_error_integral = 0.0
        self.lateral_error_previous = 0.0
        self.heading_error_integral = 0.0
        self.heading_error_previous = 0.0
        self.dt = 0.1  # Default time step
        self.path_index = 0
        
        # Error history for analysis
        self.error_history = []
    
    def find_reference_point(self, robot_pose: Tuple[float, float, float],
                           path: List[Tuple[float, float]]) -> Tuple[int, float, float, float]:
        """
        Find reference point on path and calculate tracking errors.
        
        Args:
            robot_pose: Current robot pose (x, y, theta)
            path: List of (x, y) path points
            
        Returns:
            Tuple of (path_index, lateral_error, heading_error, reference_heading)
        """
        robot_x, robot_y, robot_theta = robot_pose
        
        # Find closest point on path
        min_distance = float('inf')
        closest_index = 0
        best_lateral_error = 0.0
        best_reference_heading = 0.0
        
        # Search around current path index
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
            t = np.clip(t, 0, 1)
            
            closest_point = p1 + t * segment_vec
            distance = np.linalg.norm(robot_pos - closest_point)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
                # Calculate lateral error (cross-track error)
                cross_product = np.cross(segment_vec, to_robot)
                best_lateral_error = cross_product / segment_length
                
                # Calculate reference heading
                best_reference_heading = np.arctan2(segment_vec[1], segment_vec[0])
        
        # Update path index
        self.path_index = max(self.path_index, closest_index - 2)
        
        # Calculate heading error
        heading_error = best_reference_heading - robot_theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        return closest_index, best_lateral_error, heading_error, best_reference_heading
    
    def pid_control(self, error: float, error_integral: float, 
                   error_previous: float, kp: float, ki: float, 
                   kd: float, dt: float) -> Tuple[float, float]:
        """
        Calculate PID control output.
        
        Args:
            error: Current error
            error_integral: Accumulated integral error
            error_previous: Previous error
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            dt: Time step
            
        Returns:
            Tuple of (control_output, updated_integral)
        """
        # Proportional term
        p_term = kp * error
        
        # Integral term with windup protection
        error_integral += error * dt
        error_integral = np.clip(error_integral, 
                               -self.integral_windup_limit,
                               self.integral_windup_limit)
        i_term = ki * error_integral
        
        # Derivative term
        if dt > 0:
            error_derivative = (error - error_previous) / dt
        else:
            error_derivative = 0.0
        d_term = kd * error_derivative
        
        # Total output
        output = p_term + i_term + d_term
        output = np.clip(output, -self.max_output, self.max_output)
        
        return output, error_integral
    
    def control(self, robot: DifferentialDrive,
                path: List[Tuple[float, float]],
                desired_velocity: float = 1.0,
                dt: float = None) -> Tuple[float, float]:
        """
        Calculate control commands using PID controller.
        
        Args:
            robot: Differential drive robot
            path: List of (x, y) path points
            desired_velocity: Desired forward velocity [m/s]
            dt: Time step [s]
            
        Returns:
            Tuple of (linear_velocity, angular_velocity) commands
        """
        if dt is None:
            dt = self.dt
        
        if len(path) < 2:
            return 0.0, 0.0
        
        robot_pose = robot.get_pose()
        
        # Find reference point and calculate errors
        path_index, lateral_error, heading_error, _ = \
            self.find_reference_point(robot_pose, path)
        
        # Lateral PID control
        lateral_output, self.lateral_error_integral = self.pid_control(
            lateral_error, self.lateral_error_integral, self.lateral_error_previous,
            self.kp_lateral, self.ki_lateral, self.kd_lateral, dt)
        
        # Heading PID control
        heading_output, self.heading_error_integral = self.pid_control(
            heading_error, self.heading_error_integral, self.heading_error_previous,
            self.kp_heading, self.ki_heading, self.kd_heading, dt)
        
        # Update previous errors
        self.lateral_error_previous = lateral_error
        self.heading_error_previous = heading_error
        
        # Combine lateral and heading control
        # Lateral control affects angular velocity
        # Heading control also affects angular velocity
        angular_velocity = lateral_output + heading_output
        
        # Apply angular velocity limits
        max_angular_velocity = robot.params.max_angular_velocity
        angular_velocity = np.clip(angular_velocity,
                                 -max_angular_velocity,
                                 max_angular_velocity)
        
        # Reduce forward velocity if angular velocity is high (for stability)
        velocity_reduction_factor = 1.0 - 0.5 * abs(angular_velocity) / max_angular_velocity
        linear_velocity = desired_velocity * max(0.1, velocity_reduction_factor)
        
        # Limit velocities based on robot constraints
        linear_velocity, angular_velocity = robot.limit_velocities(
            linear_velocity, angular_velocity)
        
        # Store error history
        self.error_history.append({
            'lateral_error': lateral_error,
            'heading_error': heading_error,
            'lateral_output': lateral_output,
            'heading_output': heading_output,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity
        })
        
        # Limit history size
        if len(self.error_history) > 1000:
            self.error_history = self.error_history[-500:]
        
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
        Get current control errors.
        
        Args:
            robot: Differential drive robot
            path: List of (x, y) path points
            
        Returns:
            Tuple of (lateral_error, heading_error)
        """
        if len(path) < 2:
            return 0.0, 0.0
        
        robot_pose = robot.get_pose()
        _, lateral_error, heading_error, _ = self.find_reference_point(robot_pose, path)
        
        return lateral_error, heading_error
    
    def tune_parameters(self, **kwargs):
        """
        Tune PID parameters.
        
        Keyword Args:
            kp_lateral: Proportional gain for lateral control
            ki_lateral: Integral gain for lateral control
            kd_lateral: Derivative gain for lateral control
            kp_heading: Proportional gain for heading control
            ki_heading: Integral gain for heading control
            kd_heading: Derivative gain for heading control
        """
        for param, value in kwargs.items():
            if hasattr(self, param):
                setattr(self, param, value)
    
    def reset(self):
        """Reset controller state."""
        self.lateral_error_integral = 0.0
        self.lateral_error_previous = 0.0
        self.heading_error_integral = 0.0
        self.heading_error_previous = 0.0
        self.path_index = 0
        self.error_history = []
    
    def get_performance_metrics(self) -> dict:
        """
        Get performance metrics from error history.
        
        Returns:
            Dictionary of performance metrics
        """
        if not self.error_history:
            return {}
        
        lateral_errors = [entry['lateral_error'] for entry in self.error_history]
        heading_errors = [entry['heading_error'] for entry in self.error_history]
        
        return {
            'rms_lateral_error': np.sqrt(np.mean(np.array(lateral_errors)**2)),
            'rms_heading_error': np.sqrt(np.mean(np.array(heading_errors)**2)),
            'max_lateral_error': np.max(np.abs(lateral_errors)),
            'max_heading_error': np.max(np.abs(heading_errors)),
            'mean_lateral_error': np.mean(np.abs(lateral_errors)),
            'mean_heading_error': np.mean(np.abs(heading_errors))
        }
