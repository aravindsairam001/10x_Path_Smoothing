"""
Dynamic Window Approach (DWA) for Obstacle Avoidance
Implements the Dynamic Window Approach for local path planning and obstacle avoidance.
"""

import numpy as np
from typing import List, Tuple, Optional
import sys
import os

# Add parent directory to path for trajectory_control import
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from trajectory_control.differential_drive import DifferentialDrive, RobotState


class Obstacle:
    """Simple circular obstacle representation."""
    
    def __init__(self, x: float, y: float, radius: float):
        """
        Initialize obstacle.
        
        Args:
            x: Obstacle center x coordinate
            y: Obstacle center y coordinate
            radius: Obstacle radius
        """
        self.x = x
        self.y = y
        self.radius = radius
    
    def distance_to_point(self, x: float, y: float) -> float:
        """Calculate distance from point to obstacle surface."""
        center_distance = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return max(0, center_distance - self.radius)
    
    def is_point_inside(self, x: float, y: float, safety_margin: float = 0.0) -> bool:
        """Check if point is inside obstacle (with safety margin)."""
        distance = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return distance <= (self.radius + safety_margin)


class DynamicWindowApproach:
    """
    Dynamic Window Approach for local path planning and obstacle avoidance.
    
    The DWA algorithm selects optimal velocity commands by evaluating
    trajectories within the robot's dynamic window and choosing the one
    that best balances goal seeking, obstacle avoidance, and velocity constraints.
    """
    
    def __init__(self, robot: DifferentialDrive,
                 predict_time: float = 2.0,
                 goal_cost_gain: float = 1.0,
                 speed_cost_gain: float = 0.1,
                 obstacle_cost_gain: float = 1.0,
                 v_resolution: float = 0.1,
                 omega_resolution: float = 0.1,
                 safety_margin: float = 0.2):
        """
        Initialize Dynamic Window Approach.
        
        Args:
            robot: Differential drive robot model
            predict_time: Prediction time horizon [s]
            goal_cost_gain: Weight for goal cost
            speed_cost_gain: Weight for speed cost
            obstacle_cost_gain: Weight for obstacle cost
            v_resolution: Linear velocity resolution [m/s]
            omega_resolution: Angular velocity resolution [rad/s]
            safety_margin: Safety margin around obstacles [m]
        """
        self.robot = robot
        self.predict_time = predict_time
        self.goal_cost_gain = goal_cost_gain
        self.speed_cost_gain = speed_cost_gain
        self.obstacle_cost_gain = obstacle_cost_gain
        self.v_resolution = v_resolution
        self.omega_resolution = omega_resolution
        self.safety_margin = safety_margin
        
        # Dynamic window parameters
        self.dt = 0.1  # Time step for trajectory prediction
    
    def calculate_dynamic_window(self, current_v: float, current_omega: float) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window of admissible velocities.
        
        Args:
            current_v: Current linear velocity [m/s]
            current_omega: Current angular velocity [rad/s]
            
        Returns:
            Tuple of (v_min, v_max, omega_min, omega_max)
        """
        # Robot's absolute velocity limits
        v_min_abs = -self.robot.params.max_linear_velocity
        v_max_abs = self.robot.params.max_linear_velocity
        omega_min_abs = -self.robot.params.max_angular_velocity
        omega_max_abs = self.robot.params.max_angular_velocity
        
        # Dynamic constraints based on acceleration limits
        max_dv = self.robot.params.max_linear_acceleration * self.dt
        max_domega = self.robot.params.max_angular_acceleration * self.dt
        
        v_min_dyn = current_v - max_dv
        v_max_dyn = current_v + max_dv
        omega_min_dyn = current_omega - max_domega
        omega_max_dyn = current_omega + max_domega
        
        # Combine absolute and dynamic constraints
        v_min = max(v_min_abs, v_min_dyn)
        v_max = min(v_max_abs, v_max_dyn)
        omega_min = max(omega_min_abs, omega_min_dyn)
        omega_max = min(omega_max_abs, omega_max_dyn)
        
        return v_min, v_max, omega_min, omega_max
    
    def predict_trajectory(self, v: float, omega: float, 
                          current_state: RobotState) -> List[Tuple[float, float, float]]:
        """
        Predict robot trajectory for given velocity commands.
        
        Args:
            v: Linear velocity [m/s]
            omega: Angular velocity [rad/s]
            current_state: Current robot state
            
        Returns:
            List of predicted poses (x, y, theta)
        """
        trajectory = []
        
        # Create temporary robot for prediction
        temp_robot = DifferentialDrive(self.robot.params)
        temp_robot.set_state(current_state.x, current_state.y, current_state.theta,
                           current_state.v, current_state.omega)
        
        # Predict trajectory
        steps = int(self.predict_time / self.dt)
        for _ in range(steps):
            temp_robot.update_kinematics(v, omega, self.dt)
            pose = temp_robot.get_pose()
            trajectory.append(pose)
        
        return trajectory
    
    def calculate_goal_cost(self, trajectory: List[Tuple[float, float, float]], 
                          goal: Tuple[float, float]) -> float:
        """
        Calculate cost based on distance to goal.
        
        Args:
            trajectory: Predicted trajectory
            goal: Goal position (x, y)
            
        Returns:
            Goal cost (lower is better)
        """
        if not trajectory:
            return float('inf')
        
        # Use final position of trajectory
        final_pos = trajectory[-1]
        distance_to_goal = np.sqrt((final_pos[0] - goal[0])**2 + 
                                 (final_pos[1] - goal[1])**2)
        
        return distance_to_goal
    
    def calculate_speed_cost(self, v: float) -> float:
        """
        Calculate cost based on velocity (encourages higher speeds).
        
        Args:
            v: Linear velocity [m/s]
            
        Returns:
            Speed cost (lower is better for higher speeds)
        """
        max_speed = self.robot.params.max_linear_velocity
        return (max_speed - abs(v)) / max_speed
    
    def calculate_obstacle_cost(self, trajectory: List[Tuple[float, float, float]], 
                              obstacles: List[Obstacle]) -> float:
        """
        Calculate cost based on proximity to obstacles.
        
        Args:
            trajectory: Predicted trajectory
            obstacles: List of obstacles
            
        Returns:
            Obstacle cost (lower is better, inf for collision)
        """
        if not obstacles or not trajectory:
            return 0.0
        
        min_distance = float('inf')
        
        for x, y, _ in trajectory:
            for obstacle in obstacles:
                distance = obstacle.distance_to_point(x, y)
                
                # Check for collision
                if obstacle.is_point_inside(x, y, self.safety_margin):
                    return float('inf')  # Collision - invalid trajectory
                
                min_distance = min(min_distance, distance)
        
        # Convert distance to cost (closer = higher cost)
        if min_distance == float('inf'):
            return 0.0
        else:
            return 1.0 / (min_distance + 0.1)  # Add small value to avoid division by zero
    
    def evaluate_trajectory(self, v: float, omega: float, 
                          current_state: RobotState,
                          goal: Tuple[float, float],
                          obstacles: List[Obstacle]) -> float:
        """
        Evaluate a trajectory using the DWA cost function.
        
        Args:
            v: Linear velocity [m/s]
            omega: Angular velocity [rad/s]
            current_state: Current robot state
            goal: Goal position (x, y)
            obstacles: List of obstacles
            
        Returns:
            Total cost (lower is better)
        """
        # Predict trajectory
        trajectory = self.predict_trajectory(v, omega, current_state)
        
        # Calculate individual costs
        goal_cost = self.calculate_goal_cost(trajectory, goal)
        speed_cost = self.calculate_speed_cost(v)
        obstacle_cost = self.calculate_obstacle_cost(trajectory, obstacles)
        
        # Check for invalid trajectory
        if obstacle_cost == float('inf'):
            return float('inf')
        
        # Combine costs
        total_cost = (self.goal_cost_gain * goal_cost + 
                     self.speed_cost_gain * speed_cost + 
                     self.obstacle_cost_gain * obstacle_cost)
        
        return total_cost
    
    def plan(self, current_state: RobotState, 
             goal: Tuple[float, float],
             obstacles: List[Obstacle] = None) -> Tuple[float, float]:
        """
        Plan optimal velocity commands using DWA.
        
        Args:
            current_state: Current robot state
            goal: Goal position (x, y)
            obstacles: List of obstacles (empty list if None)
            
        Returns:
            Tuple of (optimal_v, optimal_omega)
        """
        if obstacles is None:
            obstacles = []
        
        # Calculate dynamic window
        v_min, v_max, omega_min, omega_max = self.calculate_dynamic_window(
            current_state.v, current_state.omega)
        
        # Generate velocity candidates
        v_candidates = np.arange(v_min, v_max + self.v_resolution, self.v_resolution)
        omega_candidates = np.arange(omega_min, omega_max + self.omega_resolution, 
                                   self.omega_resolution)
        
        best_v = 0.0
        best_omega = 0.0
        best_cost = float('inf')
        
        # Evaluate all velocity combinations
        for v in v_candidates:
            for omega in omega_candidates:
                # Check wheel velocity constraints
                if not self.robot.check_wheel_velocity_limits(v, omega):
                    continue
                
                cost = self.evaluate_trajectory(v, omega, current_state, goal, obstacles)
                
                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_omega = omega
        
        # If no valid trajectory found, stop
        if best_cost == float('inf'):
            return 0.0, 0.0
        
        return best_v, best_omega
    
    def get_best_trajectory(self, current_state: RobotState,
                           goal: Tuple[float, float],
                           obstacles: List[Obstacle] = None) -> List[Tuple[float, float, float]]:
        """
        Get the best predicted trajectory.
        
        Args:
            current_state: Current robot state
            goal: Goal position (x, y)
            obstacles: List of obstacles
            
        Returns:
            Best predicted trajectory
        """
        best_v, best_omega = self.plan(current_state, goal, obstacles)
        return self.predict_trajectory(best_v, best_omega, current_state)
    
    def tune_parameters(self, **kwargs):
        """
        Tune DWA parameters.
        
        Keyword Args:
            goal_cost_gain: Weight for goal cost
            speed_cost_gain: Weight for speed cost
            obstacle_cost_gain: Weight for obstacle cost
            predict_time: Prediction time horizon
            safety_margin: Safety margin around obstacles
        """
        for param, value in kwargs.items():
            if hasattr(self, param):
                setattr(self, param, value)


def create_obstacle_field(width: float, height: float, 
                         num_obstacles: int = 5,
                         min_radius: float = 0.3,
                         max_radius: float = 1.0) -> List[Obstacle]:
    """
    Create a random obstacle field for testing.
    
    Args:
        width: Field width [m]
        height: Field height [m]
        num_obstacles: Number of obstacles
        min_radius: Minimum obstacle radius [m]
        max_radius: Maximum obstacle radius [m]
        
    Returns:
        List of obstacles
    """
    obstacles = []
    
    for _ in range(num_obstacles):
        x = np.random.uniform(1, width - 1)  # Avoid edges
        y = np.random.uniform(1, height - 1)
        radius = np.random.uniform(min_radius, max_radius)
        obstacles.append(Obstacle(x, y, radius))
    
    return obstacles
