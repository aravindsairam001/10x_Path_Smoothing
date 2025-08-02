"""
Test Path Smoothing Algorithms
Unit tests for path smoothing functionality.
"""

import unittest
import numpy as np
import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_smoothing import SplineSmoother, BezierSmoother, BSplineSmoother
from utils.geometry import euclidean_distance, calculate_path_length


class TestPathSmoothing(unittest.TestCase):
    """Test cases for path smoothing algorithms."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.simple_waypoints = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)]
        self.line_waypoints = [(0, 0), (1, 0), (2, 0), (3, 0)]
        self.circle_waypoints = [
            (1, 0), (0.707, 0.707), (0, 1), (-0.707, 0.707),
            (-1, 0), (-0.707, -0.707), (0, -1), (0.707, -0.707)
        ]
    
    def test_spline_smoother_basic(self):
        """Test basic spline smoothing functionality."""
        smoother = SplineSmoother()
        
        # Test with simple waypoints
        x_smooth, y_smooth = smoother.smooth(self.simple_waypoints, num_points=50)
        
        self.assertEqual(len(x_smooth), 50)
        self.assertEqual(len(y_smooth), 50)
        
        # Check that smoothed path starts and ends at correct points
        self.assertAlmostEqual(x_smooth[0], self.simple_waypoints[0][0], places=2)
        self.assertAlmostEqual(y_smooth[0], self.simple_waypoints[0][1], places=2)
        self.assertAlmostEqual(x_smooth[-1], self.simple_waypoints[-1][0], places=2)
        self.assertAlmostEqual(y_smooth[-1], self.simple_waypoints[-1][1], places=2)
    
    def test_bezier_smoother_basic(self):
        """Test basic Bezier smoothing functionality."""
        smoother = BezierSmoother()
        
        # Test with simple waypoints
        x_smooth, y_smooth = smoother.smooth(self.simple_waypoints, num_points=50)
        
        self.assertEqual(len(x_smooth), 50)
        self.assertEqual(len(y_smooth), 50)
        
        # Check start and end points
        self.assertAlmostEqual(x_smooth[0], self.simple_waypoints[0][0], places=2)
        self.assertAlmostEqual(y_smooth[0], self.simple_waypoints[0][1], places=2)
    
    def test_bspline_smoother_basic(self):
        """Test basic B-spline smoothing functionality."""
        smoother = BSplineSmoother()
        
        # Test with simple waypoints
        x_smooth, y_smooth = smoother.smooth(self.simple_waypoints, num_points=50)
        
        self.assertEqual(len(x_smooth), 50)
        self.assertEqual(len(y_smooth), 50)
    
    def test_curvature_calculation(self):
        """Test curvature calculation."""
        smoother = SplineSmoother()
        
        # Test with circular waypoints (should have relatively constant curvature)
        curvature = smoother.compute_curvature(self.circle_waypoints, num_points=100)
        
        self.assertEqual(len(curvature), 100)
        self.assertTrue(np.all(curvature >= 0))  # Curvature should be non-negative
        
        # For a circle with radius ~1, curvature should be ~1
        mean_curvature = np.mean(curvature)
        self.assertGreater(mean_curvature, 0.5)
        self.assertLess(mean_curvature, 2.0)
    
    def test_path_length_calculation(self):
        """Test path length calculation."""
        smoother = SplineSmoother()
        
        # Test with line waypoints (known length)
        length = smoother.compute_path_length(self.line_waypoints, num_points=100)
        
        # Should be approximately 3 units (0->3 on x-axis)
        self.assertAlmostEqual(length, 3.0, places=1)
    
    def test_edge_cases(self):
        """Test edge cases."""
        smoother = SplineSmoother()
        
        # Test with too few waypoints
        with self.assertRaises(ValueError):
            smoother.smooth([(0, 0)], num_points=10)
        
        # Test with two waypoints (should work)
        x_smooth, y_smooth = smoother.smooth([(0, 0), (1, 1)], num_points=10)
        self.assertEqual(len(x_smooth), 10)
        self.assertEqual(len(y_smooth), 10)
    
    def test_smoothing_consistency(self):
        """Test that different smoothers produce reasonable results."""
        smoothers = {
            'spline': SplineSmoother(),
            'bezier': BezierSmoother(),
            'bspline': BSplineSmoother()
        }
        
        results = {}
        for name, smoother in smoothers.items():
            try:
                x_smooth, y_smooth = smoother.smooth(self.simple_waypoints, num_points=50)
                results[name] = {
                    'length': len(x_smooth),
                    'start': (x_smooth[0], y_smooth[0]),
                    'end': (x_smooth[-1], y_smooth[-1])
                }
            except Exception as e:
                self.fail(f"Smoother {name} failed: {e}")
        
        # All should produce same number of points
        lengths = [result['length'] for result in results.values()]
        self.assertTrue(all(length == 50 for length in lengths))
        
        # All should start at approximately the same point
        starts = [result['start'] for result in results.values()]
        for start in starts:
            self.assertAlmostEqual(start[0], self.simple_waypoints[0][0], places=1)
            self.assertAlmostEqual(start[1], self.simple_waypoints[0][1], places=1)


class TestGeometryUtils(unittest.TestCase):
    """Test cases for geometry utility functions."""
    
    def test_euclidean_distance(self):
        """Test Euclidean distance calculation."""
        # Test simple cases
        self.assertEqual(euclidean_distance((0, 0), (3, 4)), 5.0)
        self.assertEqual(euclidean_distance((0, 0), (0, 0)), 0.0)
        self.assertAlmostEqual(euclidean_distance((1, 1), (2, 2)), np.sqrt(2), places=5)
    
    def test_calculate_path_length(self):
        """Test path length calculation."""
        # Test simple path
        path = [(0, 0), (1, 0), (1, 1), (0, 1)]
        length = calculate_path_length(path)
        self.assertEqual(length, 3.0)  # 1 + 1 + 1
        
        # Test empty path
        self.assertEqual(calculate_path_length([]), 0.0)
        
        # Test single point
        self.assertEqual(calculate_path_length([(0, 0)]), 0.0)


if __name__ == '__main__':
    unittest.main()
