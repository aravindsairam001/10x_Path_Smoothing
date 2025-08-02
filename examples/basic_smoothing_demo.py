"""
Basic Path Smoothing Demo
Demonstrates different path smoothing algorithms.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_smoothing import SplineSmoother, BezierSmoother, BSplineSmoother
from utils.geometry import generate_circle_waypoints, generate_spiral_waypoints


def create_test_waypoints():
    """Create various test waypoint patterns."""
    # Simple path with sharp turns
    waypoints_1 = [(0, 0), (2, 0), (4, 2), (6, 2), (8, 0), (10, 0)]
    
    # S-curve path
    waypoints_2 = [(0, 0), (2, 1), (4, -1), (6, 1), (8, -1), (10, 0)]
    
    # Circular path
    waypoints_3 = generate_circle_waypoints((5, 5), 3, num_points=8)
    
    # Spiral path
    waypoints_4 = generate_spiral_waypoints((0, 0), 1, 4, num_turns=1.5, num_points=10)
    
    return {
        'Sharp Turns': waypoints_1,
        'S-Curve': waypoints_2,
        'Circle': waypoints_3,
        'Spiral': waypoints_4
    }


def demonstrate_smoothing_algorithms():
    """Demonstrate different smoothing algorithms."""
    waypoint_sets = create_test_waypoints()
    
    # Initialize smoothers
    spline_smoother = SplineSmoother()
    bezier_smoother = BezierSmoother()
    bspline_smoother = BSplineSmoother()
    
    # Create subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    for idx, (name, waypoints) in enumerate(waypoint_sets.items()):
        ax = axes[idx]
        
        # Original waypoints
        waypoints_array = np.array(waypoints)
        ax.plot(waypoints_array[:, 0], waypoints_array[:, 1], 'ko-', 
                markersize=8, linewidth=2, label='Original Waypoints')
        
        try:
            # Spline smoothing
            x_spline, y_spline = spline_smoother.smooth(waypoints, num_points=100)
            ax.plot(x_spline, y_spline, 'b-', linewidth=2, label='Cubic Spline')
            
            # Bezier smoothing
            x_bezier, y_bezier = bezier_smoother.smooth(waypoints, num_points=100)
            ax.plot(x_bezier, y_bezier, 'r--', linewidth=2, label='Bezier Curves')
            
            # B-spline smoothing
            x_bspline, y_bspline = bspline_smoother.smooth(waypoints, num_points=100)
            ax.plot(x_bspline, y_bspline, 'g:', linewidth=2, label='B-Spline')
            
        except Exception as e:
            print(f"Error smoothing {name}: {e}")
        
        ax.set_title(f'{name} Path Smoothing')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig('path_smoothing_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def analyze_smoothing_properties():
    """Analyze properties of different smoothing methods."""
    # Create a test path with varying complexity
    waypoints = [(0, 0), (1, 2), (3, 1), (5, 3), (7, 0), (9, 2), (10, 1)]
    
    # Initialize smoothers
    smoothers = {
        'Cubic Spline': SplineSmoother(),
        'Bezier Curves': BezierSmoother(),
        'B-Spline': BSplineSmoother()
    }
    
    print("Path Smoothing Analysis")
    print("=" * 50)
    
    for name, smoother in smoothers.items():
        try:
            if hasattr(smoother, 'get_path_info'):
                info = smoother.get_path_info(waypoints, num_points=100)
                print(f"\n{name}:")
                print(f"  Path Length: {info['path_length']:.3f} m")
                print(f"  Max Curvature: {info['max_curvature']:.3f} rad/m")
                print(f"  Mean Curvature: {info['mean_curvature']:.3f} rad/m")
            else:
                # Basic analysis for spline smoother
                x_smooth, y_smooth = smoother.smooth(waypoints, num_points=100)
                path_length = smoother.compute_path_length(waypoints, num_points=100)
                curvature = smoother.compute_curvature(waypoints, num_points=100)
                print(f"\n{name}:")
                print(f"  Path Length: {path_length:.3f} m")
                print(f"  Max Curvature: {np.max(curvature):.3f} rad/m")
                print(f"  Mean Curvature: {np.mean(curvature):.3f} rad/m")
                
        except Exception as e:
            print(f"\nError analyzing {name}: {e}")


def plot_curvature_analysis():
    """Plot curvature analysis for different smoothing methods."""
    waypoints = [(0, 0), (2, 0), (4, 2), (6, 2), (8, 0), (10, 0)]
    
    # Initialize smoothers
    spline_smoother = SplineSmoother()
    bezier_smoother = BezierSmoother()
    bspline_smoother = BSplineSmoother()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot paths
    waypoints_array = np.array(waypoints)
    ax1.plot(waypoints_array[:, 0], waypoints_array[:, 1], 'ko-', 
             markersize=8, linewidth=2, label='Original Waypoints')
    
    try:
        # Spline smoothing and curvature
        x_spline, y_spline = spline_smoother.smooth(waypoints, num_points=100)
        curvature_spline = spline_smoother.compute_curvature(waypoints, num_points=100)
        ax1.plot(x_spline, y_spline, 'b-', linewidth=2, label='Cubic Spline')
        
        # Bezier smoothing and curvature
        x_bezier, y_bezier = bezier_smoother.smooth(waypoints, num_points=100)
        bezier_info = bezier_smoother.get_path_info(waypoints, num_points=100)
        curvature_bezier = bezier_info['curvature']
        ax1.plot(x_bezier, y_bezier, 'r--', linewidth=2, label='Bezier Curves')
        
        # B-spline smoothing and curvature
        x_bspline, y_bspline = bspline_smoother.smooth(waypoints, num_points=100)
        curvature_bspline = bspline_smoother.compute_curvature(waypoints, num_points=100)
        ax1.plot(x_bspline, y_bspline, 'g:', linewidth=2, label='B-Spline')
        
        # Plot curvatures
        s_spline = np.linspace(0, 1, len(curvature_spline))
        s_bezier = np.linspace(0, 1, len(curvature_bezier))
        s_bspline = np.linspace(0, 1, len(curvature_bspline))
        
        ax2.plot(s_spline, curvature_spline, 'b-', linewidth=2, label='Cubic Spline')
        ax2.plot(s_bezier, curvature_bezier, 'r--', linewidth=2, label='Bezier Curves')
        ax2.plot(s_bspline, curvature_bspline, 'g:', linewidth=2, label='B-Spline')
        
    except Exception as e:
        print(f"Error in curvature analysis: {e}")
    
    ax1.set_title('Path Comparison')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    ax2.set_title('Curvature Analysis')
    ax2.set_xlabel('Normalized Path Parameter')
    ax2.set_ylabel('Curvature [rad/m]')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('curvature_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()


def main():
    """Main demonstration function."""
    print("Path Smoothing Demo")
    print("==================")
    
    # Demonstrate different smoothing algorithms
    print("\n1. Comparing smoothing algorithms...")
    demonstrate_smoothing_algorithms()
    
    # Analyze smoothing properties
    print("\n2. Analyzing smoothing properties...")
    analyze_smoothing_properties()
    
    # Plot curvature analysis
    print("\n3. Curvature analysis...")
    plot_curvature_analysis()
    
    print("\nDemo completed! Check generated plots:")
    print("  - path_smoothing_comparison.png")
    print("  - curvature_analysis.png")


if __name__ == "__main__":
    main()
