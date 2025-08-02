"""
Spline-based Path Smoothing
Implements cubic spline interpolation for smooth path generation.
"""

import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from typing import List, Tuple, Optional


class SplineSmoother:
    """
    Cubic spline-based path smoother for converting discrete waypoints 
    into smooth, continuous trajectories.
    """
    
    def __init__(self, smoothing_factor: float = 0.0, boundary_conditions: str = 'natural'):
        """
        Initialize the spline smoother.
        
        Args:
            smoothing_factor: Smoothing factor for spline (0 = no smoothing)
            boundary_conditions: Boundary conditions for spline ('natural', 'clamped', 'not-a-knot')
        """
        self.smoothing_factor = smoothing_factor
        self.boundary_conditions = boundary_conditions
    
    def smooth(self, waypoints: List[Tuple[float, float]], 
               num_points: int = 100,
               parametric: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate smooth path from discrete waypoints using cubic splines.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points in the smoothed path
            parametric: Whether to use parametric spline interpolation
            
        Returns:
            Tuple of (x_smooth, y_smooth) arrays representing the smooth path
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints are required")
        
        waypoints = np.array(waypoints)
        x_waypoints = waypoints[:, 0]
        y_waypoints = waypoints[:, 1]
        
        if parametric:
            return self._parametric_spline(x_waypoints, y_waypoints, num_points)
        else:
            return self._direct_spline(x_waypoints, y_waypoints, num_points)
    
    def _parametric_spline(self, x_waypoints: np.ndarray, 
                          y_waypoints: np.ndarray, 
                          num_points: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Parametric cubic spline interpolation.
        """
        # Calculate cumulative distances along the path
        distances = np.cumsum(np.sqrt(np.diff(x_waypoints)**2 + np.diff(y_waypoints)**2))
        distances = np.insert(distances, 0, 0)  # Insert 0 at the beginning
        
        # Create cubic splines for x and y as functions of distance
        cs_x = CubicSpline(distances, x_waypoints, bc_type=self.boundary_conditions)
        cs_y = CubicSpline(distances, y_waypoints, bc_type=self.boundary_conditions)
        
        # Generate smooth parameter values
        t_smooth = np.linspace(0, distances[-1], num_points)
        
        # Evaluate splines at smooth parameter values
        x_smooth = cs_x(t_smooth)
        y_smooth = cs_y(t_smooth)
        
        return x_smooth, y_smooth
    
    def _direct_spline(self, x_waypoints: np.ndarray, 
                      y_waypoints: np.ndarray, 
                      num_points: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Direct cubic spline interpolation (y as function of x).
        """
        # Sort waypoints by x-coordinate
        sorted_indices = np.argsort(x_waypoints)
        x_sorted = x_waypoints[sorted_indices]
        y_sorted = y_waypoints[sorted_indices]
        
        # Create cubic spline
        cs = CubicSpline(x_sorted, y_sorted, bc_type=self.boundary_conditions)
        
        # Generate smooth x values
        x_smooth = np.linspace(x_sorted[0], x_sorted[-1], num_points)
        y_smooth = cs(x_smooth)
        
        return x_smooth, y_smooth
    
    def compute_curvature(self, waypoints: List[Tuple[float, float]], 
                         num_points: int = 100) -> np.ndarray:
        """
        Compute curvature along the smoothed path.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points for curvature calculation
            
        Returns:
            Array of curvature values along the path
        """
        x_smooth, y_smooth = self.smooth(waypoints, num_points)
        
        # Compute first and second derivatives
        dx = np.gradient(x_smooth)
        dy = np.gradient(y_smooth)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # Compute curvature: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        numerator = np.abs(dx * ddy - dy * ddx)
        denominator = (dx**2 + dy**2)**(3/2)
        
        # Avoid division by zero
        denominator = np.where(denominator == 0, 1e-8, denominator)
        curvature = numerator / denominator
        
        return curvature
    
    def compute_path_length(self, waypoints: List[Tuple[float, float]], 
                           num_points: int = 100) -> float:
        """
        Compute total length of the smoothed path.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points for length calculation
            
        Returns:
            Total path length
        """
        x_smooth, y_smooth = self.smooth(waypoints, num_points)
        
        # Compute distances between consecutive points
        dx = np.diff(x_smooth)
        dy = np.diff(y_smooth)
        distances = np.sqrt(dx**2 + dy**2)
        
        return np.sum(distances)
    
    def get_path_derivatives(self, waypoints: List[Tuple[float, float]], 
                           num_points: int = 100) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get first and second derivatives of the smoothed path.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points for derivative calculation
            
        Returns:
            Tuple of (dx, dy, ddx, ddy) derivative arrays
        """
        x_smooth, y_smooth = self.smooth(waypoints, num_points)
        
        # Compute first derivatives
        dx = np.gradient(x_smooth)
        dy = np.gradient(y_smooth)
        
        # Compute second derivatives
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        return dx, dy, ddx, ddy
