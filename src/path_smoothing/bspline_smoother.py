"""
B-Spline Path Smoothing
Implements B-spline curve generation for smooth path creation.
"""

import numpy as np
from scipy.interpolate import splprep, splev, BSpline
from typing import List, Tuple, Optional


class BSplineSmoother:
    """
    B-spline based path smoother for converting discrete waypoints 
    into smooth, continuous trajectories.
    """
    
    def __init__(self, degree: int = 3, smoothing: float = 0.0):
        """
        Initialize the B-spline smoother.
        
        Args:
            degree: Degree of the B-spline (usually 3 for cubic)
            smoothing: Smoothing factor (0 = interpolation, >0 = approximation)
        """
        self.degree = degree
        self.smoothing = smoothing
    
    def smooth(self, waypoints: List[Tuple[float, float]], 
               num_points: int = 100,
               periodic: bool = False) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate smooth path from discrete waypoints using B-splines.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points in the smoothed path
            periodic: Whether the path should be periodic (closed loop)
            
        Returns:
            Tuple of (x_smooth, y_smooth) arrays representing the smooth path
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints are required")
        
        waypoints = np.array(waypoints)
        
        if len(waypoints) <= self.degree:
            # Not enough points for B-spline, use linear interpolation
            return self._linear_interpolation(waypoints, num_points)
        
        # Separate x and y coordinates
        x_waypoints = waypoints[:, 0]
        y_waypoints = waypoints[:, 1]
        
        try:
            # Fit parametric B-spline
            tck, u = splprep([x_waypoints, y_waypoints], 
                           k=self.degree, 
                           s=self.smoothing,
                           per=periodic)
            
            # Generate smooth parameter values
            u_smooth = np.linspace(0, 1, num_points)
            
            # Evaluate B-spline at smooth parameter values
            smooth_path = splev(u_smooth, tck)
            x_smooth, y_smooth = smooth_path[0], smooth_path[1]
            
            return x_smooth, y_smooth
            
        except Exception as e:
            print(f"B-spline fitting failed: {e}, falling back to linear interpolation")
            return self._linear_interpolation(waypoints, num_points)
    
    def _linear_interpolation(self, waypoints: np.ndarray, 
                            num_points: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fallback linear interpolation when B-spline fails.
        """
        # Calculate cumulative distances
        distances = np.cumsum(np.sqrt(np.diff(waypoints[:, 0])**2 + 
                                    np.diff(waypoints[:, 1])**2))
        distances = np.insert(distances, 0, 0)
        
        # Interpolate
        t_smooth = np.linspace(0, distances[-1], num_points)
        x_smooth = np.interp(t_smooth, distances, waypoints[:, 0])
        y_smooth = np.interp(t_smooth, distances, waypoints[:, 1])
        
        return x_smooth, y_smooth
    
    def smooth_with_constraints(self, waypoints: List[Tuple[float, float]], 
                              constraints: List[Tuple[int, float, float]] = None,
                              num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate smooth path with derivative constraints at specific points.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            constraints: List of (point_index, dx, dy) derivative constraints
            num_points: Number of points in the smoothed path
            
        Returns:
            Tuple of (x_smooth, y_smooth) arrays representing the smooth path
        """
        if constraints is None:
            return self.smooth(waypoints, num_points)
        
        waypoints = np.array(waypoints)
        
        # TODO: Implement constraint handling for B-splines
        # For now, use basic smoothing
        return self.smooth(waypoints.tolist(), num_points)
    
    def compute_derivatives(self, waypoints: List[Tuple[float, float]], 
                          num_points: int = 100,
                          derivative_order: int = 1) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Compute derivatives of the B-spline path.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points for derivative calculation
            derivative_order: Maximum order of derivatives to compute
            
        Returns:
            List of derivative tuples [(dx, dy), (ddx, ddy), ...]
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints are required")
        
        waypoints = np.array(waypoints)
        x_waypoints = waypoints[:, 0]
        y_waypoints = waypoints[:, 1]
        
        if len(waypoints) <= self.degree:
            # Use numerical derivatives for insufficient points
            x_smooth, y_smooth = self._linear_interpolation(waypoints, num_points)
            derivatives = []
            
            dx, dy = x_smooth, y_smooth
            for order in range(1, derivative_order + 1):
                dx = np.gradient(dx)
                dy = np.gradient(dy)
                derivatives.append((dx.copy(), dy.copy()))
            
            return derivatives
        
        try:
            # Fit parametric B-spline
            tck, u = splprep([x_waypoints, y_waypoints], 
                           k=self.degree, 
                           s=self.smoothing)
            
            # Generate smooth parameter values
            u_smooth = np.linspace(0, 1, num_points)
            
            # Compute derivatives
            derivatives = []
            for order in range(1, derivative_order + 1):
                deriv = splev(u_smooth, tck, der=order)
                derivatives.append((deriv[0], deriv[1]))
            
            return derivatives
            
        except Exception as e:
            print(f"B-spline derivative computation failed: {e}")
            return []
    
    def compute_curvature(self, waypoints: List[Tuple[float, float]], 
                         num_points: int = 100) -> np.ndarray:
        """
        Compute curvature along the B-spline path.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points for curvature calculation
            
        Returns:
            Array of curvature values along the path
        """
        derivatives = self.compute_derivatives(waypoints, num_points, derivative_order=2)
        
        if len(derivatives) < 2:
            return np.zeros(num_points)
        
        dx, dy = derivatives[0]  # First derivative
        ddx, ddy = derivatives[1]  # Second derivative
        
        # Compute curvature: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        numerator = np.abs(dx * ddy - dy * ddx)
        denominator = (dx**2 + dy**2)**(3/2)
        
        # Avoid division by zero
        denominator = np.where(denominator == 0, 1e-8, denominator)
        curvature = numerator / denominator
        
        return curvature
    
    def adaptive_smoothing(self, waypoints: List[Tuple[float, float]], 
                          max_curvature: float = 1.0,
                          num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        Adaptive smoothing that adjusts parameters based on curvature constraints.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            max_curvature: Maximum allowed curvature
            num_points: Number of points in the smoothed path
            
        Returns:
            Tuple of (x_smooth, y_smooth) arrays representing the smooth path
        """
        best_smoothing = self.smoothing
        best_path = None
        
        # Try different smoothing factors
        smoothing_factors = [0.0, 0.1, 0.5, 1.0, 2.0, 5.0]
        
        for smoothing in smoothing_factors:
            temp_smoother = BSplineSmoother(self.degree, smoothing)
            try:
                x_smooth, y_smooth = temp_smoother.smooth(waypoints, num_points)
                curvature = temp_smoother.compute_curvature(waypoints, num_points)
                
                if np.max(curvature) <= max_curvature:
                    best_path = (x_smooth, y_smooth)
                    best_smoothing = smoothing
                    break
                    
            except Exception:
                continue
        
        if best_path is None:
            # If no valid smoothing found, use the highest smoothing factor
            temp_smoother = BSplineSmoother(self.degree, smoothing_factors[-1])
            best_path = temp_smoother.smooth(waypoints, num_points)
        
        return best_path
    
    def get_path_info(self, waypoints: List[Tuple[float, float]], 
                     num_points: int = 100) -> dict:
        """
        Get comprehensive information about the B-spline path.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points for analysis
            
        Returns:
            Dictionary containing path information
        """
        x_smooth, y_smooth = self.smooth(waypoints, num_points)
        curvature = self.compute_curvature(waypoints, num_points)
        
        # Compute path length
        dx = np.diff(x_smooth)
        dy = np.diff(y_smooth)
        distances = np.sqrt(dx**2 + dy**2)
        total_length = np.sum(distances)
        
        return {
            'path_length': total_length,
            'max_curvature': np.max(curvature) if len(curvature) > 0 else 0,
            'mean_curvature': np.mean(curvature) if len(curvature) > 0 else 0,
            'curvature': curvature,
            'x_smooth': x_smooth,
            'y_smooth': y_smooth,
            'smoothing_factor': self.smoothing,
            'degree': self.degree
        }
