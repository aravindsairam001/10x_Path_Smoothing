"""
Main Application for Path Smoothing and Trajectory Control
Demonstrates complete path smoothing and robot control pipeline.
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from path_smoothing import SplineSmoother, BezierSmoother, BSplineSmoother
from trajectory_control import DifferentialDrive, PurePursuitController, StanleyController, PIDController
from utils.geometry import (
    generate_circle_waypoints, generate_figure_eight_waypoints, 
    generate_spiral_waypoints, calculate_path_length
)


class PathSmoothingApp:
    """Main application class for path smoothing and trajectory control."""
    
    def __init__(self):
        """Initialize the application."""
        self.smoothers = {
            'spline': SplineSmoother(),
            'bezier': BezierSmoother(),
            'bspline': BSplineSmoother()
        }
        
        self.controllers = {
            'pure_pursuit': PurePursuitController(lookahead_distance=2.0),
            'stanley': StanleyController(k_e=0.5),
            'pid': PIDController(kp_lateral=1.0, kp_heading=2.0)
        }
        
        self.robot = DifferentialDrive()
    
    def generate_test_waypoints(self, pattern='simple'):
        """Generate test waypoints based on pattern."""
        patterns = {
            'simple': [(0, 0), (2, 1), (4, 3), (6, 4), (8, 3), (10, 1), (12, 0)],
            'sharp_turns': [(0, 0), (2, 0), (4, 2), (6, 2), (8, 0), (10, 0)],
            's_curve': [(0, 0), (2, 1), (4, -1), (6, 1), (8, -1), (10, 0)],
            'circle': generate_circle_waypoints((5, 5), 3, num_points=8),
            'figure_eight': generate_figure_eight_waypoints((5, 5), 3, num_points=16),
            'spiral': generate_spiral_waypoints((0, 0), 1, 4, num_turns=2, num_points=12)
        }
        
        return patterns.get(pattern, patterns['simple'])
    
    def smooth_path(self, waypoints, method='spline', num_points=100):
        """Smooth waypoints using specified method."""
        if method not in self.smoothers:
            raise ValueError(f"Unknown smoothing method: {method}")
        
        smoother = self.smoothers[method]
        x_smooth, y_smooth = smoother.smooth(waypoints, num_points)
        
        return list(zip(x_smooth, y_smooth))
    
    def simulate_trajectory_following(self, trajectory, controller='pure_pursuit', 
                                    desired_velocity=2.0, sim_time=15.0):
        """Simulate robot following trajectory."""
        if controller not in self.controllers:
            raise ValueError(f"Unknown controller: {controller}")
        
        # Reset robot and controller
        self.robot.set_state(trajectory[0][0], trajectory[0][1], 0)
        ctrl = self.controllers[controller]
        ctrl.reset()
        
        # Simulation parameters
        dt = 0.1
        steps = int(sim_time / dt)
        
        # Storage
        robot_positions = []
        control_commands = []
        tracking_errors = []
        
        for step in range(steps):
            robot_pose = self.robot.get_pose()
            robot_positions.append(robot_pose)
            
            # Check if goal is reached
            if ctrl.is_goal_reached(robot_pose, trajectory[-1], tolerance=0.5):
                print(f"Goal reached at step {step}!")
                break
            
            try:
                # Get control commands
                v_cmd, omega_cmd = ctrl.control(self.robot, trajectory, desired_velocity)
                control_commands.append((v_cmd, omega_cmd))
                
                # Calculate tracking error
                min_distance = min(np.linalg.norm(np.array(robot_pose[:2]) - np.array(point)) 
                                 for point in trajectory)
                tracking_errors.append(min_distance)
                
                # Update robot
                self.robot.update_kinematics(v_cmd, omega_cmd, dt)
                
            except Exception as e:
                print(f"Simulation error: {e}")
                break
        
        return {
            'positions': robot_positions,
            'commands': control_commands,
            'errors': tracking_errors
        }
    
    def plot_complete_pipeline(self, waypoints, smoothing_method='spline', 
                             controller='pure_pursuit'):
        """Plot complete pipeline from waypoints to trajectory following."""
        # Smooth the path
        smooth_trajectory = self.smooth_path(waypoints, smoothing_method, num_points=150)
        
        # Simulate trajectory following
        sim_results = self.simulate_trajectory_following(
            smooth_trajectory, controller, desired_velocity=2.0)
        
        # Create visualization
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        
        # Plot 1: Path smoothing
        ax1 = axes[0, 0]
        waypoints_array = np.array(waypoints)
        trajectory_array = np.array(smooth_trajectory)
        
        ax1.plot(waypoints_array[:, 0], waypoints_array[:, 1], 'ko-', 
                markersize=8, linewidth=2, label='Original Waypoints')
        ax1.plot(trajectory_array[:, 0], trajectory_array[:, 1], 'b-', 
                linewidth=2, label=f'{smoothing_method.title()} Smoothed')
        
        ax1.set_title('Path Smoothing')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot 2: Trajectory following
        ax2 = axes[0, 1]
        positions = np.array([pos[:2] for pos in sim_results['positions']])
        
        ax2.plot(trajectory_array[:, 0], trajectory_array[:, 1], 'k-', 
                linewidth=2, alpha=0.7, label='Reference')
        ax2.plot(positions[:, 0], positions[:, 1], 'r-', 
                linewidth=2, label=f'{controller.title()} Controller')
        ax2.plot(positions[0, 0], positions[0, 1], 'go', markersize=8, label='Start')
        ax2.plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=8, label='End')
        
        ax2.set_title('Trajectory Following')
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Y [m]')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # Plot 3: Control commands
        ax3 = axes[1, 0]
        commands = np.array(sim_results['commands'])
        time_steps = np.arange(len(commands)) * 0.1
        
        ax3.plot(time_steps, commands[:, 0], 'b-', linewidth=2, label='Linear Velocity')
        ax3.plot(time_steps, commands[:, 1], 'r-', linewidth=2, label='Angular Velocity')
        
        ax3.set_title('Control Commands')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Velocity [m/s, rad/s]')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Tracking errors
        ax4 = axes[1, 1]
        errors = sim_results['errors']
        time_steps = np.arange(len(errors)) * 0.1
        
        ax4.plot(time_steps, errors, 'g-', linewidth=2)
        ax4.set_title('Tracking Error')
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Distance Error [m]')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        filename = f'complete_pipeline_{smoothing_method}_{controller}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {filename}")
        
        plt.show()
        
        return smooth_trajectory, sim_results
    
    def run_performance_comparison(self, waypoint_pattern='simple'):
        """Run comprehensive performance comparison."""
        waypoints = self.generate_test_waypoints(waypoint_pattern)
        
        print(f"Performance Comparison - {waypoint_pattern.title()} Path")
        print("=" * 60)
        
        results = {}
        
        for smooth_method in self.smoothers.keys():
            for controller in self.controllers.keys():
                try:
                    # Generate smooth trajectory
                    trajectory = self.smooth_path(waypoints, smooth_method)
                    
                    # Simulate trajectory following
                    sim_result = self.simulate_trajectory_following(
                        trajectory, controller, desired_velocity=2.0)
                    
                    # Calculate metrics
                    errors = sim_result['errors']
                    rms_error = np.sqrt(np.mean(np.array(errors)**2)) if errors else float('inf')
                    max_error = np.max(errors) if errors else float('inf')
                    final_error = errors[-1] if errors else float('inf')
                    
                    results[(smooth_method, controller)] = {
                        'rms_error': rms_error,
                        'max_error': max_error,
                        'final_error': final_error,
                        'completion_time': len(sim_result['positions']) * 0.1
                    }
                    
                    print(f"{smooth_method:>8} + {controller:>12}: "
                          f"RMS={rms_error:.3f}m, Max={max_error:.3f}m, "
                          f"Final={final_error:.3f}m, Time={results[(smooth_method, controller)]['completion_time']:.1f}s")
                
                except Exception as e:
                    print(f"Error with {smooth_method}+{controller}: {e}")
        
        return results


def main():
    """Main function with command line interface."""
    parser = argparse.ArgumentParser(description='Path Smoothing and Trajectory Control Demo')
    parser.add_argument('--pattern', default='simple', 
                       choices=['simple', 'sharp_turns', 's_curve', 'circle', 'figure_eight', 'spiral'],
                       help='Waypoint pattern to use')
    parser.add_argument('--smoother', default='spline',
                       choices=['spline', 'bezier', 'bspline'],
                       help='Path smoothing method')
    parser.add_argument('--controller', default='pure_pursuit',
                       choices=['pure_pursuit', 'stanley', 'pid'],
                       help='Trajectory following controller')
    parser.add_argument('--compare', action='store_true',
                       help='Run performance comparison of all methods')
    
    args = parser.parse_args()
    
    # Initialize application
    app = PathSmoothingApp()
    
    print("Path Smoothing and Trajectory Control System")
    print("=" * 50)
    
    if args.compare:
        # Run performance comparison
        app.run_performance_comparison(args.pattern)
    else:
        # Generate waypoints
        waypoints = app.generate_test_waypoints(args.pattern)
        print(f"Generated {len(waypoints)} waypoints for {args.pattern} pattern")
        
        # Run complete pipeline
        print(f"Using {args.smoother} smoother and {args.controller} controller")
        
        try:
            trajectory, results = app.plot_complete_pipeline(
                waypoints, args.smoother, args.controller)
            
            # Print summary
            if results['errors']:
                print(f"\nPerformance Summary:")
                print(f"  RMS Tracking Error: {np.sqrt(np.mean(np.array(results['errors'])**2)):.3f} m")
                print(f"  Max Tracking Error: {np.max(results['errors']):.3f} m")
                print(f"  Final Tracking Error: {results['errors'][-1]:.3f} m")
                print(f"  Simulation Steps: {len(results['positions'])}")
                print(f"  Simulation Time: {len(results['positions']) * 0.1:.1f} s")
            
        except Exception as e:
            print(f"Error running pipeline: {e}")
            sys.exit(1)
    
    print("\nDemo completed successfully!")


if __name__ == "__main__":
    main()
