"""
Utility functions for geometry and path operations.
"""

import numpy as np
from typing import List, Tuple, Optional


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-pi, pi]
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """
    Calculate Euclidean distance between two points.
    
    Args:
        p1: First point (x, y)
        p2: Second point (x, y)
        
    Returns:
        Euclidean distance
    """
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def calculate_path_length(path: List[Tuple[float, float]]) -> float:
    """
    Calculate total length of a path.
    
    Args:
        path: List of (x, y) points
        
    Returns:
        Total path length
    """
    if len(path) < 2:
        return 0.0
    
    total_length = 0.0
    for i in range(len(path) - 1):
        total_length += euclidean_distance(path[i], path[i + 1])
    
    return total_length


def resample_path(path: List[Tuple[float, float]], 
                  target_spacing: float) -> List[Tuple[float, float]]:
    """
    Resample path with uniform spacing.
    
    Args:
        path: Original path points
        target_spacing: Desired spacing between points
        
    Returns:
        Resampled path with uniform spacing
    """
    if len(path) < 2:
        return path
    
    resampled_path = [path[0]]  # Start with first point
    current_distance = 0.0
    
    for i in range(len(path) - 1):
        segment_start = np.array(path[i])
        segment_end = np.array(path[i + 1])
        segment_length = np.linalg.norm(segment_end - segment_start)
        
        # Add points along this segment
        while current_distance + target_spacing <= segment_length:
            current_distance += target_spacing
            t = current_distance / segment_length
            new_point = segment_start + t * (segment_end - segment_start)
            resampled_path.append(tuple(new_point))
        
        current_distance -= segment_length
    
    # Ensure last point is included
    if resampled_path[-1] != path[-1]:
        resampled_path.append(path[-1])
    
    return resampled_path


def calculate_heading_along_path(path: List[Tuple[float, float]]) -> List[float]:
    """
    Calculate heading angle at each point along the path.
    
    Args:
        path: List of (x, y) path points
        
    Returns:
        List of heading angles in radians
    """
    if len(path) < 2:
        return [0.0] * len(path)
    
    headings = []
    
    for i in range(len(path)):
        if i == 0:
            # First point: use direction to next point
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
        elif i == len(path) - 1:
            # Last point: use direction from previous point
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
        else:
            # Middle points: use average direction
            dx1 = path[i][0] - path[i - 1][0]
            dy1 = path[i][1] - path[i - 1][1]
            dx2 = path[i + 1][0] - path[i][0]
            dy2 = path[i + 1][1] - path[i][1]
            dx = (dx1 + dx2) / 2
            dy = (dy1 + dy2) / 2
        
        heading = np.arctan2(dy, dx)
        headings.append(heading)
    
    return headings


def calculate_curvature_along_path(path: List[Tuple[float, float]]) -> List[float]:
    """
    Calculate curvature at each point along the path.
    
    Args:
        path: List of (x, y) path points
        
    Returns:
        List of curvature values [rad/m]
    """
    if len(path) < 3:
        return [0.0] * len(path)
    
    curvatures = [0.0]  # First point has zero curvature
    
    for i in range(1, len(path) - 1):
        # Three consecutive points
        p1 = np.array(path[i - 1])
        p2 = np.array(path[i])
        p3 = np.array(path[i + 1])
        
        # Vectors
        v1 = p2 - p1
        v2 = p3 - p2
        
        # Calculate curvature using cross product formula
        cross_product = np.cross(v1, v2)
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm > 1e-6 and v2_norm > 1e-6:
            # Curvature = |v1 × v2| / |v1|³
            curvature = abs(cross_product) / (v1_norm**3)
        else:
            curvature = 0.0
        
        curvatures.append(curvature)
    
    curvatures.append(0.0)  # Last point has zero curvature
    
    return curvatures


def point_to_line_distance(point: Tuple[float, float],
                          line_start: Tuple[float, float],
                          line_end: Tuple[float, float]) -> float:
    """
    Calculate distance from point to line segment.
    
    Args:
        point: Point coordinates (x, y)
        line_start: Line segment start (x, y)
        line_end: Line segment end (x, y)
        
    Returns:
        Distance from point to line segment
    """
    p = np.array(point)
    a = np.array(line_start)
    b = np.array(line_end)
    
    # Vector from a to b
    ab = b - a
    ab_length_squared = np.dot(ab, ab)
    
    if ab_length_squared == 0:
        # Line segment is a point
        return np.linalg.norm(p - a)
    
    # Project point onto line segment
    t = np.dot(p - a, ab) / ab_length_squared
    t = np.clip(t, 0, 1)  # Clamp to segment
    
    # Closest point on segment
    closest_point = a + t * ab
    
    return np.linalg.norm(p - closest_point)


def generate_circle_waypoints(center: Tuple[float, float], 
                            radius: float,
                            num_points: int = 20,
                            start_angle: float = 0.0) -> List[Tuple[float, float]]:
    """
    Generate waypoints along a circular path.
    
    Args:
        center: Circle center (x, y)
        radius: Circle radius
        num_points: Number of waypoints
        start_angle: Starting angle in radians
        
    Returns:
        List of waypoints along the circle
    """
    waypoints = []
    angles = np.linspace(start_angle, start_angle + 2 * np.pi, num_points + 1)[:-1]
    
    for angle in angles:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        waypoints.append((x, y))
    
    return waypoints


def generate_figure_eight_waypoints(center: Tuple[float, float],
                                  radius: float,
                                  num_points: int = 40) -> List[Tuple[float, float]]:
    """
    Generate waypoints along a figure-eight path.
    
    Args:
        center: Path center (x, y)
        radius: Path radius
        num_points: Number of waypoints
        
    Returns:
        List of waypoints along figure-eight
    """
    waypoints = []
    t_values = np.linspace(0, 2 * np.pi, num_points + 1)[:-1]
    
    for t in t_values:
        # Parametric equations for figure-eight (lemniscate)
        x = center[0] + radius * np.cos(t) / (1 + np.sin(t)**2)
        y = center[1] + radius * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)
        waypoints.append((x, y))
    
    return waypoints


def generate_spiral_waypoints(center: Tuple[float, float],
                            start_radius: float,
                            end_radius: float,
                            num_turns: float = 2.0,
                            num_points: int = 50) -> List[Tuple[float, float]]:
    """
    Generate waypoints along a spiral path.
    
    Args:
        center: Spiral center (x, y)
        start_radius: Starting radius
        end_radius: Ending radius
        num_turns: Number of spiral turns
        num_points: Number of waypoints
        
    Returns:
        List of waypoints along spiral
    """
    waypoints = []
    angles = np.linspace(0, num_turns * 2 * np.pi, num_points)
    radii = np.linspace(start_radius, end_radius, num_points)
    
    for angle, radius in zip(angles, radii):
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        waypoints.append((x, y))
    
    return waypoints


def transform_path(path: List[Tuple[float, float]], 
                  translation: Tuple[float, float] = (0, 0),
                  rotation: float = 0.0,
                  scale: float = 1.0) -> List[Tuple[float, float]]:
    """
    Apply geometric transformation to a path.
    
    Args:
        path: Original path points
        translation: Translation offset (dx, dy)
        rotation: Rotation angle in radians
        scale: Scaling factor
        
    Returns:
        Transformed path
    """
    transformed_path = []
    cos_rot = np.cos(rotation)
    sin_rot = np.sin(rotation)
    
    for x, y in path:
        # Scale
        x_scaled = x * scale
        y_scaled = y * scale
        
        # Rotate
        x_rotated = x_scaled * cos_rot - y_scaled * sin_rot
        y_rotated = x_scaled * sin_rot + y_scaled * cos_rot
        
        # Translate
        x_final = x_rotated + translation[0]
        y_final = y_rotated + translation[1]
        
        transformed_path.append((x_final, y_final))
    
    return transformed_path
