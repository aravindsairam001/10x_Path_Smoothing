#!/usr/bin/env python3
"""
Cubic Spline Path Smoothing for ROS2 and TurtleBot3
"""

import numpy as np
from scipy.interpolate import CubicSpline
from typing import List, Tuple
import math


class CubicSplinePathSmoother:
    """
    Cubic spline path smoother that takes waypoints and generates a smooth path
    """
    
    def __init__(self, resolution: float = 0.1):
        """
        Initialize the cubic spline path smoother
        
        Args:
            resolution: Distance between points in the smoothed path (meters)
        """
        self.resolution = resolution
        self.smoothed_path = []
        self.path_length = 0.0
    
    def smooth_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth the given waypoints using cubic spline interpolation
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            
        Returns:
            List of smoothed path points as (x, y) tuples
        """
        if len(waypoints) < 2:
            return waypoints
        
        if len(waypoints) == 2:
            # Linear interpolation for two points
            return self._linear_interpolation(waypoints[0], waypoints[1])
        
        # Extract x and y coordinates
        x_coords = [p[0] for p in waypoints]
        y_coords = [p[1] for p in waypoints]
        
        # Calculate cumulative distances for parameterization
        distances = [0.0]
        for i in range(1, len(waypoints)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            distances.append(distances[-1] + math.sqrt(dx*dx + dy*dy))
        
        # Create cubic splines for x and y coordinates
        try:
            cs_x = CubicSpline(distances, x_coords)
            cs_y = CubicSpline(distances, y_coords)
        except Exception as e:
            print(f"Error creating cubic spline: {e}")
            return waypoints
        
        # Generate smoothed path points
        total_distance = distances[-1]
        self.path_length = total_distance
        num_points = max(len(waypoints), int(total_distance / self.resolution))
        t_values = np.linspace(0, total_distance, num_points)
        
        smoothed = []
        for t in t_values:
            x = float(cs_x(t))
            y = float(cs_y(t))
            smoothed.append((x, y))
        
        self.smoothed_path = smoothed
        return smoothed
    
    def _linear_interpolation(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Linear interpolation between two points
        """
        distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        num_points = max(2, int(distance / self.resolution))
        
        smoothed = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            smoothed.append((x, y))
        
        return smoothed
    
    def get_path_heading(self, index: int) -> float:
        """
        Get the heading angle at a specific point in the smoothed path
        
        Args:
            index: Index of the point in the smoothed path
            
        Returns:
            Heading angle in radians
        """
        if not self.smoothed_path or index >= len(self.smoothed_path) - 1:
            return 0.0
        
        current = self.smoothed_path[index]
        next_point = self.smoothed_path[index + 1]
        
        dx = next_point[0] - current[0]
        dy = next_point[1] - current[1]
        
        return math.atan2(dy, dx)
    
    def get_curvature(self, index: int) -> float:
        """
        Calculate the curvature at a specific point in the smoothed path
        
        Args:
            index: Index of the point in the smoothed path
            
        Returns:
            Curvature value (1/radius)
        """
        if not self.smoothed_path or index == 0 or index >= len(self.smoothed_path) - 1:
            return 0.0
        
        # Use three consecutive points to calculate curvature
        p1 = self.smoothed_path[index - 1]
        p2 = self.smoothed_path[index]
        p3 = self.smoothed_path[index + 1]
        
        # Calculate vectors
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        
        # Calculate cross product and magnitudes
        cross_product = v1[0] * v2[1] - v1[1] * v2[0]
        mag_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
        mag_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
        
        # Avoid division by zero
        if mag_v1 < 1e-6 or mag_v2 < 1e-6:
            return 0.0
        
        # Calculate curvature
        curvature = 2 * cross_product / (mag_v1 * mag_v2 * (mag_v1 + mag_v2))
        
        return curvature
