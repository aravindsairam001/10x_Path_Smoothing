"""
Differential Drive Robot Model
Models the kinematics and dynamics of a differential drive robot.
"""

import numpy as np
from typing import Tuple, List
from dataclasses import dataclass


@dataclass
class RobotState:
    """Robot state representation."""
    x: float = 0.0  # Position x [m]
    y: float = 0.0  # Position y [m]
    theta: float = 0.0  # Orientation [rad]
    v: float = 0.0  # Linear velocity [m/s]
    omega: float = 0.0  # Angular velocity [rad/s]


@dataclass  
class RobotParameters:
    """Robot physical parameters."""
    wheelbase: float = 0.5  # Distance between wheels [m]
    max_linear_velocity: float = 2.0  # Maximum linear velocity [m/s]
    max_angular_velocity: float = 2.0  # Maximum angular velocity [rad/s]
    max_linear_acceleration: float = 1.0  # Maximum linear acceleration [m/s²]
    max_angular_acceleration: float = 1.0  # Maximum angular acceleration [rad/s²]
    wheel_radius: float = 0.05  # Wheel radius [m]


class DifferentialDrive:
    """
    Differential drive robot model with kinematic and dynamic constraints.
    """
    
    def __init__(self, parameters: RobotParameters = None):
        """
        Initialize differential drive robot.
        
        Args:
            parameters: Robot physical parameters
        """
        self.params = parameters if parameters else RobotParameters()
        self.state = RobotState()
        self.dt = 0.1  # Default time step [s]
        
    def set_state(self, x: float, y: float, theta: float, 
                  v: float = 0.0, omega: float = 0.0):
        """
        Set robot state.
        
        Args:
            x: Position x [m]
            y: Position y [m]
            theta: Orientation [rad]
            v: Linear velocity [m/s]
            omega: Angular velocity [rad/s]
        """
        self.state.x = x
        self.state.y = y
        self.state.theta = theta
        self.state.v = v
        self.state.omega = omega
    
    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get current robot pose.
        
        Returns:
            Tuple of (x, y, theta)
        """
        return self.state.x, self.state.y, self.state.theta
    
    def get_state(self) -> RobotState:
        """
        Get current robot state.
        
        Returns:
            Current robot state
        """
        return self.state
    
    def update_kinematics(self, v_cmd: float, omega_cmd: float, dt: float = None):
        """
        Update robot state using kinematic model.
        
        Args:
            v_cmd: Commanded linear velocity [m/s]
            omega_cmd: Commanded angular velocity [rad/s]
            dt: Time step [s]
        """
        if dt is None:
            dt = self.dt
        
        # Apply velocity constraints
        v_cmd = np.clip(v_cmd, -self.params.max_linear_velocity, 
                       self.params.max_linear_velocity)
        omega_cmd = np.clip(omega_cmd, -self.params.max_angular_velocity,
                           self.params.max_angular_velocity)
        
        # Kinematic model
        self.state.x += v_cmd * np.cos(self.state.theta) * dt
        self.state.y += v_cmd * np.sin(self.state.theta) * dt
        self.state.theta += omega_cmd * dt
        
        # Normalize theta to [-pi, pi]
        self.state.theta = np.arctan2(np.sin(self.state.theta), 
                                     np.cos(self.state.theta))
        
        # Update velocities
        self.state.v = v_cmd
        self.state.omega = omega_cmd
    
    def update_dynamics(self, v_cmd: float, omega_cmd: float, dt: float = None):
        """
        Update robot state using dynamic model with acceleration limits.
        
        Args:
            v_cmd: Commanded linear velocity [m/s]
            omega_cmd: Commanded angular velocity [rad/s]
            dt: Time step [s]
        """
        if dt is None:
            dt = self.dt
        
        # Apply acceleration constraints
        dv = v_cmd - self.state.v
        domega = omega_cmd - self.state.omega
        
        max_dv = self.params.max_linear_acceleration * dt
        max_domega = self.params.max_angular_acceleration * dt
        
        dv = np.clip(dv, -max_dv, max_dv)
        domega = np.clip(domega, -max_domega, max_domega)
        
        # Update velocities with acceleration constraints
        new_v = self.state.v + dv
        new_omega = self.state.omega + domega
        
        # Apply velocity constraints
        new_v = np.clip(new_v, -self.params.max_linear_velocity,
                       self.params.max_linear_velocity)
        new_omega = np.clip(new_omega, -self.params.max_angular_velocity,
                           self.params.max_angular_velocity)
        
        # Update position using average velocity
        avg_v = (self.state.v + new_v) / 2
        avg_omega = (self.state.omega + new_omega) / 2
        
        self.state.x += avg_v * np.cos(self.state.theta + avg_omega * dt / 2) * dt
        self.state.y += avg_v * np.sin(self.state.theta + avg_omega * dt / 2) * dt
        self.state.theta += avg_omega * dt
        
        # Normalize theta
        self.state.theta = np.arctan2(np.sin(self.state.theta),
                                     np.cos(self.state.theta))
        
        # Update velocities
        self.state.v = new_v
        self.state.omega = new_omega
    
    def wheel_velocities_to_body_velocities(self, v_left: float, v_right: float) -> Tuple[float, float]:
        """
        Convert wheel velocities to body frame velocities.
        
        Args:
            v_left: Left wheel velocity [m/s]
            v_right: Right wheel velocity [m/s]
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.params.wheelbase
        
        return v, omega
    
    def body_velocities_to_wheel_velocities(self, v: float, omega: float) -> Tuple[float, float]:
        """
        Convert body frame velocities to wheel velocities.
        
        Args:
            v: Linear velocity [m/s]
            omega: Angular velocity [rad/s]
            
        Returns:
            Tuple of (left_wheel_velocity, right_wheel_velocity)
        """
        v_left = v - omega * self.params.wheelbase / 2
        v_right = v + omega * self.params.wheelbase / 2
        
        return v_left, v_right
    
    def check_wheel_velocity_limits(self, v: float, omega: float) -> bool:
        """
        Check if commanded velocities are within wheel velocity limits.
        
        Args:
            v: Linear velocity [m/s]
            omega: Angular velocity [rad/s]
            
        Returns:
            True if within limits, False otherwise
        """
        v_left, v_right = self.body_velocities_to_wheel_velocities(v, omega)
        max_wheel_velocity = self.params.max_linear_velocity
        
        return (abs(v_left) <= max_wheel_velocity and 
                abs(v_right) <= max_wheel_velocity)
    
    def limit_velocities(self, v: float, omega: float) -> Tuple[float, float]:
        """
        Limit velocities to respect wheel velocity constraints.
        
        Args:
            v: Desired linear velocity [m/s]
            omega: Desired angular velocity [rad/s]
            
        Returns:
            Tuple of (limited_v, limited_omega)
        """
        v_left, v_right = self.body_velocities_to_wheel_velocities(v, omega)
        max_wheel_velocity = self.params.max_linear_velocity
        
        # Find the maximum scaling factor needed
        scale_factor = 1.0
        if abs(v_left) > max_wheel_velocity:
            scale_factor = min(scale_factor, max_wheel_velocity / abs(v_left))
        if abs(v_right) > max_wheel_velocity:
            scale_factor = min(scale_factor, max_wheel_velocity / abs(v_right))
        
        # Scale down velocities proportionally
        v_limited = v * scale_factor
        omega_limited = omega * scale_factor
        
        return v_limited, omega_limited
    
    def simulate_trajectory(self, v_commands: List[float], 
                          omega_commands: List[float],
                          dt: float = 0.1,
                          use_dynamics: bool = False) -> List[RobotState]:
        """
        Simulate robot trajectory given velocity commands.
        
        Args:
            v_commands: List of linear velocity commands [m/s]
            omega_commands: List of angular velocity commands [rad/s]
            dt: Time step [s]
            use_dynamics: Whether to use dynamic model
            
        Returns:
            List of robot states along trajectory
        """
        if len(v_commands) != len(omega_commands):
            raise ValueError("v_commands and omega_commands must have same length")
        
        trajectory = [RobotState(self.state.x, self.state.y, self.state.theta,
                                self.state.v, self.state.omega)]
        
        for v_cmd, omega_cmd in zip(v_commands, omega_commands):
            if use_dynamics:
                self.update_dynamics(v_cmd, omega_cmd, dt)
            else:
                self.update_kinematics(v_cmd, omega_cmd, dt)
            
            trajectory.append(RobotState(self.state.x, self.state.y, self.state.theta,
                                       self.state.v, self.state.omega))
        
        return trajectory
    
    def get_position_history(self) -> List[Tuple[float, float]]:
        """
        Get history of robot positions (for visualization).
        
        Returns:
            List of (x, y) positions
        """
        # This would be implemented with a position history buffer
        # For now, just return current position
        return [(self.state.x, self.state.y)]
