"""
Bezier Curve Path Smoothing
Implements Bezier curve generation for smooth path creation.
"""

import numpy as np
from typing import List, Tuple, Optional


class BezierSmoother:
    """
    Bezier curve-based path smoother for converting discrete waypoints 
    into smooth, continuous trajectories.
    """
    
    def __init__(self, control_point_ratio: float = 0.3):
        """
        Initialize the Bezier smoother.
        
        Args:
            control_point_ratio: Ratio for automatic control point generation
        """
        self.control_point_ratio = control_point_ratio
    
    def smooth(self, waypoints: List[Tuple[float, float]], 
               num_points: int = 100,
               control_points: Optional[List[Tuple[float, float]]] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate smooth path from discrete waypoints using Bezier curves.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points in the smoothed path
            control_points: Optional control points for Bezier curves
            
        Returns:
            Tuple of (x_smooth, y_smooth) arrays representing the smooth path
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints are required")
        
        waypoints = np.array(waypoints)
        
        if len(waypoints) == 2:
            # Simple linear interpolation for 2 points
            return self._linear_interpolation(waypoints, num_points)
        
        # Generate piecewise Bezier curves
        return self._piecewise_bezier(waypoints, num_points, control_points)
    
    def _linear_interpolation(self, waypoints: np.ndarray, 
                            num_points: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Linear interpolation between two points.
        """
        t = np.linspace(0, 1, num_points)
        x_smooth = waypoints[0, 0] + t * (waypoints[1, 0] - waypoints[0, 0])
        y_smooth = waypoints[0, 1] + t * (waypoints[1, 1] - waypoints[0, 1])
        return x_smooth, y_smooth
    
    def _piecewise_bezier(self, waypoints: np.ndarray, 
                         num_points: int,
                         control_points: Optional[List[Tuple[float, float]]] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate piecewise cubic Bezier curves through waypoints.
        """
        n_segments = len(waypoints) - 1
        points_per_segment = num_points // n_segments
        remaining_points = num_points % n_segments
        
        x_smooth = []
        y_smooth = []
        
        if control_points is None:
            control_points = self._generate_control_points(waypoints)
        
        for i in range(n_segments):
            segment_points = points_per_segment + (1 if i < remaining_points else 0)
            
            # Define Bezier control points for this segment
            P0 = waypoints[i]
            P3 = waypoints[i + 1]
            P1 = control_points[2 * i] if 2 * i < len(control_points) else P0
            P2 = control_points[2 * i + 1] if 2 * i + 1 < len(control_points) else P3
            
            # Generate Bezier curve points
            if i == 0:
                t = np.linspace(0, 1, segment_points)
            else:
                t = np.linspace(0, 1, segment_points + 1)[1:]  # Skip first point to avoid duplication
            
            x_seg, y_seg = self._cubic_bezier(P0, P1, P2, P3, t)
            
            x_smooth.extend(x_seg)
            y_smooth.extend(y_seg)
        
        # Ensure we have exactly num_points by adjusting if necessary
        while len(x_smooth) > num_points:
            x_smooth.pop()
            y_smooth.pop()
        while len(x_smooth) < num_points:
            x_smooth.append(x_smooth[-1])
            y_smooth.append(y_smooth[-1])
        
        return np.array(x_smooth), np.array(y_smooth)
    
    def _generate_control_points(self, waypoints: np.ndarray) -> List[Tuple[float, float]]:
        """
        Automatically generate control points for smooth Bezier curves.
        """
        n_points = len(waypoints)
        control_points = []
        
        for i in range(n_points - 1):
            # Calculate tangent direction
            if i == 0:
                # First segment: use direction to next point
                tangent = waypoints[i + 1] - waypoints[i]
            elif i == n_points - 2:
                # Last segment: use direction from previous point
                tangent = waypoints[i + 1] - waypoints[i]
            else:
                # Middle segments: use average direction
                tangent = waypoints[i + 1] - waypoints[i - 1]
            
            # Normalize tangent
            tangent_length = np.linalg.norm(tangent)
            if tangent_length > 0:
                tangent = tangent / tangent_length
            
            # Calculate distance between current waypoints
            segment_length = np.linalg.norm(waypoints[i + 1] - waypoints[i])
            control_distance = segment_length * self.control_point_ratio
            
            # Generate control points
            P1 = waypoints[i] + control_distance * tangent
            P2 = waypoints[i + 1] - control_distance * tangent
            
            control_points.extend([tuple(P1), tuple(P2)])
        
        return control_points
    
    def _cubic_bezier(self, P0: np.ndarray, P1: np.ndarray, 
                     P2: np.ndarray, P3: np.ndarray, 
                     t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Evaluate cubic Bezier curve at parameter values t.
        
        B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
        """
        t = t.reshape(-1, 1)  # Make t a column vector for broadcasting
        
        # Bezier basis functions
        B0 = (1 - t)**3
        B1 = 3 * (1 - t)**2 * t
        B2 = 3 * (1 - t) * t**2
        B3 = t**3
        
        # Evaluate curve
        curve_points = B0 * P0 + B1 * P1 + B2 * P2 + B3 * P3
        
        return curve_points[:, 0], curve_points[:, 1]
    
    def compute_bezier_derivative(self, P0: np.ndarray, P1: np.ndarray,
                                P2: np.ndarray, P3: np.ndarray,
                                t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute first derivative of cubic Bezier curve.
        
        B'(t) = 3(1-t)²(P1-P0) + 6(1-t)t(P2-P1) + 3t²(P3-P2)
        """
        t = t.reshape(-1, 1)
        
        # Derivative basis functions
        dB0 = 3 * (1 - t)**2
        dB1 = 6 * (1 - t) * t
        dB2 = 3 * t**2
        
        # Evaluate derivative
        derivative = dB0 * (P1 - P0) + dB1 * (P2 - P1) + dB2 * (P3 - P2)
        
        return derivative[:, 0], derivative[:, 1]
    
    def get_path_info(self, waypoints: List[Tuple[float, float]], 
                     num_points: int = 100) -> dict:
        """
        Get comprehensive information about the Bezier path.
        
        Args:
            waypoints: List of (x, y) waypoint coordinates
            num_points: Number of points for analysis
            
        Returns:
            Dictionary containing path information
        """
        x_smooth, y_smooth = self.smooth(waypoints, num_points)
        
        # Compute path length
        dx = np.diff(x_smooth)
        dy = np.diff(y_smooth)
        distances = np.sqrt(dx**2 + dy**2)
        total_length = np.sum(distances)
        
        # Compute curvature approximation
        dx_dt = np.gradient(x_smooth)
        dy_dt = np.gradient(y_smooth)
        ddx_dt2 = np.gradient(dx_dt)
        ddy_dt2 = np.gradient(dy_dt)
        
        curvature = np.abs(dx_dt * ddy_dt2 - dy_dt * ddx_dt2) / (dx_dt**2 + dy_dt**2)**(3/2)
        curvature = np.nan_to_num(curvature)  # Handle division by zero
        
        return {
            'path_length': total_length,
            'max_curvature': np.max(curvature),
            'mean_curvature': np.mean(curvature),
            'curvature': curvature,
            'x_smooth': x_smooth,
            'y_smooth': y_smooth
        }
