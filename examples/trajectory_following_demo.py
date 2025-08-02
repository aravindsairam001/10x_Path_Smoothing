"""
Trajectory Following Demo
Demonstrates trajectory following using different control algorithms.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_smoothing import SplineSmoother
from trajectory_control import DifferentialDrive, PurePursuitController, StanleyController, PIDController
from utils.geometry import generate_circle_waypoints


def create_test_trajectory():
    """Create a test trajectory for robot following."""
    # Create waypoints for a complex path
    waypoints = [
        (0, 0), (2, 1), (4, 3), (6, 4), (8, 3), 
        (10, 1), (12, 0), (14, -1), (16, 0), (18, 2)
    ]
    
    # Smooth the path
    smoother = SplineSmoother()
    x_smooth, y_smooth = smoother.smooth(waypoints, num_points=200)
    
    # Convert to list of tuples
    trajectory = list(zip(x_smooth, y_smooth))
    
    return waypoints, trajectory


def simulate_trajectory_following():
    """Simulate trajectory following with different controllers."""
    # Create test trajectory
    waypoints, trajectory = create_test_trajectory()
    
    # Initialize robot
    robot = DifferentialDrive()
    robot.set_state(0, 0, 0)  # Start at origin
    
    # Initialize controllers
    controllers = {
        'Pure Pursuit': PurePursuitController(lookahead_distance=2.0),
        'Stanley': StanleyController(k_e=0.5),
        'PID': PIDController(kp_lateral=1.0, kp_heading=2.0)
    }
    
    # Simulation parameters
    dt = 0.1
    desired_velocity = 2.0
    sim_time = 20.0
    steps = int(sim_time / dt)
    
    # Storage for results
    results = {}
    
    for controller_name, controller in controllers.items():
        print(f"Simulating {controller_name} controller...")
        
        # Reset robot and controller
        robot.set_state(0, 0, 0)
        controller.reset()
        
        # Storage for this controller
        robot_positions = []
        control_commands = []
        errors = []
        
        for step in range(steps):
            # Get current robot pose
            robot_pose = robot.get_pose()
            robot_positions.append(robot_pose[:2])  # Store (x, y)
            
            # Calculate control commands
            try:
                v_cmd, omega_cmd = controller.control(robot, trajectory, desired_velocity)
                control_commands.append((v_cmd, omega_cmd))
                
                # Get tracking errors if available
                if hasattr(controller, 'get_control_errors'):
                    lateral_error, heading_error = controller.get_control_errors(robot, trajectory)
                    errors.append((lateral_error, heading_error))
                else:
                    errors.append((0, 0))
                
                # Update robot state
                robot.update_kinematics(v_cmd, omega_cmd, dt)
                
            except Exception as e:
                print(f"Error in {controller_name}: {e}")
                break
        
        results[controller_name] = {
            'positions': robot_positions,
            'commands': control_commands,
            'errors': errors
        }
    
    return waypoints, trajectory, results


def plot_trajectory_comparison():
    """Plot comparison of different trajectory following controllers."""
    waypoints, trajectory, results = simulate_trajectory_following()
    
    # Create the plot
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # Plot 1: Trajectory comparison
    ax1 = axes[0, 0]
    
    # Plot reference trajectory
    traj_array = np.array(trajectory)
    waypoints_array = np.array(waypoints)
    ax1.plot(traj_array[:, 0], traj_array[:, 1], 'k-', linewidth=2, label='Reference')
    ax1.plot(waypoints_array[:, 0], waypoints_array[:, 1], 'ko', markersize=6, label='Waypoints')
    
    # Plot robot trajectories
    colors = ['blue', 'red', 'green']
    for i, (controller_name, data) in enumerate(results.items()):
        positions = np.array(data['positions'])
        if len(positions) > 0:
            ax1.plot(positions[:, 0], positions[:, 1], color=colors[i], 
                    linewidth=2, alpha=0.7, label=f'{controller_name}')
    
    ax1.set_title('Trajectory Following Comparison')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Control commands comparison
    ax2 = axes[0, 1]
    
    for i, (controller_name, data) in enumerate(results.items()):
        commands = np.array(data['commands'])
        if len(commands) > 0:
            time_steps = np.arange(len(commands)) * 0.1
            ax2.plot(time_steps, commands[:, 0], color=colors[i], 
                    linewidth=2, label=f'{controller_name} - Linear')
            ax2.plot(time_steps, commands[:, 1], color=colors[i], 
                    linewidth=2, linestyle='--', alpha=0.7, label=f'{controller_name} - Angular')
    
    ax2.set_title('Control Commands')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [m/s or rad/s]')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Lateral errors
    ax3 = axes[1, 0]
    
    for i, (controller_name, data) in enumerate(results.items()):
        errors = np.array(data['errors'])
        if len(errors) > 0:
            time_steps = np.arange(len(errors)) * 0.1
            ax3.plot(time_steps, np.abs(errors[:, 0]), color=colors[i], 
                    linewidth=2, label=f'{controller_name}')
    
    ax3.set_title('Lateral Tracking Error')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('|Lateral Error| [m]')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Heading errors
    ax4 = axes[1, 1]
    
    for i, (controller_name, data) in enumerate(results.items()):
        errors = np.array(data['errors'])
        if len(errors) > 0:
            time_steps = np.arange(len(errors)) * 0.1
            ax4.plot(time_steps, np.abs(errors[:, 1]), color=colors[i], 
                    linewidth=2, label=f'{controller_name}')
    
    ax4.set_title('Heading Tracking Error')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('|Heading Error| [rad]')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_following_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def analyze_controller_performance():
    """Analyze and compare controller performance metrics."""
    waypoints, trajectory, results = simulate_trajectory_following()
    
    print("\nController Performance Analysis")
    print("=" * 50)
    
    for controller_name, data in results.items():
        positions = np.array(data['positions'])
        errors = np.array(data['errors'])
        
        if len(positions) == 0:
            continue
        
        # Calculate performance metrics
        lateral_errors = errors[:, 0] if len(errors) > 0 else []
        heading_errors = errors[:, 1] if len(errors) > 0 else []
        
        # Calculate tracking accuracy
        if len(positions) > 0 and len(trajectory) > 0:
            # Find closest trajectory point for each robot position
            tracking_errors = []
            for pos in positions:
                distances = [np.linalg.norm(np.array(pos) - np.array(traj_point)) 
                           for traj_point in trajectory]
                tracking_errors.append(min(distances))
            
            print(f"\n{controller_name} Controller:")
            print(f"  Average Tracking Error: {np.mean(tracking_errors):.3f} m")
            print(f"  Max Tracking Error: {np.max(tracking_errors):.3f} m")
            print(f"  RMS Tracking Error: {np.sqrt(np.mean(np.array(tracking_errors)**2)):.3f} m")
            
            if len(lateral_errors) > 0:
                print(f"  Average Lateral Error: {np.mean(np.abs(lateral_errors)):.3f} m")
                print(f"  RMS Lateral Error: {np.sqrt(np.mean(lateral_errors**2)):.3f} m")
            
            if len(heading_errors) > 0:
                print(f"  Average Heading Error: {np.mean(np.abs(heading_errors)):.3f} rad")
                print(f"  RMS Heading Error: {np.sqrt(np.mean(heading_errors**2)):.3f} rad")


def create_animated_simulation():
    """Create an animated visualization of trajectory following."""
    waypoints, trajectory, results = simulate_trajectory_following()
    
    # Set up the figure
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Plot reference trajectory
    traj_array = np.array(trajectory)
    waypoints_array = np.array(waypoints)
    ax.plot(traj_array[:, 0], traj_array[:, 1], 'k-', linewidth=2, label='Reference')
    ax.plot(waypoints_array[:, 0], waypoints_array[:, 1], 'ko', markersize=6, label='Waypoints')
    
    # Initialize robot markers
    colors = ['blue', 'red', 'green']
    robot_markers = []
    robot_trails = []
    
    for i, (controller_name, data) in enumerate(results.items()):
        marker, = ax.plot([], [], 'o', color=colors[i], markersize=8, label=f'{controller_name}')
        trail, = ax.plot([], [], color=colors[i], alpha=0.5, linewidth=1)
        robot_markers.append(marker)
        robot_trails.append(trail)
    
    ax.set_xlim(-2, 20)
    ax.set_ylim(-3, 6)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Animated Trajectory Following')
    
    def animate(frame):
        for i, ((controller_name, data), marker, trail) in enumerate(zip(results.items(), robot_markers, robot_trails)):
            positions = data['positions']
            if frame < len(positions):
                # Update robot position
                x, y = positions[frame]
                marker.set_data([x], [y])
                
                # Update trail
                trail_positions = positions[:frame+1]
                if trail_positions:
                    trail_x, trail_y = zip(*trail_positions)
                    trail.set_data(trail_x, trail_y)
        
        return robot_markers + robot_trails
    
    # Create animation
    max_frames = max(len(data['positions']) for data in results.values())
    anim = animation.FuncAnimation(fig, animate, frames=max_frames, 
                                 interval=50, blit=True, repeat=True)
    
    # Save animation
    try:
        anim.save('trajectory_following_animation.gif', writer='pillow', fps=20)
        print("Animation saved as trajectory_following_animation.gif")
    except Exception as e:
        print(f"Could not save animation: {e}")
    
    plt.show()


def main():
    """Main demonstration function."""
    print("Trajectory Following Demo")
    print("========================")
    
    # Plot trajectory comparison
    print("\n1. Comparing trajectory following controllers...")
    plot_trajectory_comparison()
    
    # Analyze controller performance
    print("\n2. Analyzing controller performance...")
    analyze_controller_performance()
    
    # Create animated simulation
    print("\n3. Creating animated simulation...")
    create_animated_simulation()
    
    print("\nDemo completed! Check generated files:")
    print("  - trajectory_following_comparison.png")
    print("  - trajectory_following_animation.gif")


if __name__ == "__main__":
    main()
