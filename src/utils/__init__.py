"""
Utility Package
Provides helper functions for geometry, visualization, and analysis.
"""

from .geometry import (
    normalize_angle, euclidean_distance, calculate_path_length,
    resample_path, calculate_heading_along_path, calculate_curvature_along_path,
    point_to_line_distance, generate_circle_waypoints, generate_figure_eight_waypoints,
    generate_spiral_waypoints, transform_path
)

__all__ = [
    'normalize_angle', 'euclidean_distance', 'calculate_path_length',
    'resample_path', 'calculate_heading_along_path', 'calculate_curvature_along_path',
    'point_to_line_distance', 'generate_circle_waypoints', 'generate_figure_eight_waypoints',
    'generate_spiral_waypoints', 'transform_path'
]
