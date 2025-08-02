"""
Obstacle Avoidance Demo
Demonstrates dynamic obstacle avoidance using the Dynamic Window Approach.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_smoothing import SplineSmoother
from trajectory_control import DifferentialDrive, PurePursuitController
from obstacle_avoidance import DynamicWindowApproach
from obstacle_avoidance.dynamic_window import Obstacle, create_obstacle_field


def create_test_scenario():
    """Create a test scenario with obstacles."""
    # Define start and goal
    start = (1, 1, 0)  # x, y, theta
    goal = (18, 8)
    
    # Create obstacles
    obstacles = [
        Obstacle(5, 3, 0.8),
        Obstacle(8, 6, 1.0),
        Obstacle(12, 2, 0.6),
        Obstacle(15, 7, 0.9),
        Obstacle(10, 4, 0.7),
        Obstacle(6, 8, 0.5),
        Obstacle(14, 5, 0.8)
    ]
    
    return start, goal, obstacles


def simulate_obstacle_avoidance():
    """Simulate robot navigation with obstacle avoidance."""
    # Create test scenario
    start, goal, obstacles = create_test_scenario()
    
    # Initialize robot
    robot = DifferentialDrive()
    robot.set_state(start[0], start[1], start[2])
    
    # Initialize DWA planner
    dwa = DynamicWindowApproach(
        robot=robot,
        predict_time=1.5,
        goal_cost_gain=1.0,
        speed_cost_gain=0.1,
        obstacle_cost_gain=2.0,
        v_resolution=0.2,
        omega_resolution=0.2,
        safety_margin=0.3
    )
    
    # Simulation parameters
    dt = 0.1
    max_time = 30.0
    steps = int(max_time / dt)
    goal_threshold = 0.5
    
    # Storage
    robot_trajectory = []
    control_commands = []
    predicted_trajectories = []
    
    print("Starting obstacle avoidance simulation...")
    
    for step in range(steps):
        current_state = robot.get_state()
        current_pose = robot.get_pose()
        robot_trajectory.append(current_pose)
        
        # Check if goal is reached
        distance_to_goal = np.sqrt((current_pose[0] - goal[0])**2 + 
                                 (current_pose[1] - goal[1])**2)
        
        if distance_to_goal < goal_threshold:
            print(f"Goal reached at step {step}! Distance: {distance_to_goal:.3f}m")
            break
        
        # Plan using DWA
        try:
            best_v, best_omega = dwa.plan(current_state, goal, obstacles)
            control_commands.append((best_v, best_omega))
            
            # Get predicted trajectory for visualization
            pred_trajectory = dwa.get_best_trajectory(current_state, goal, obstacles)
            predicted_trajectories.append(pred_trajectory)
            
            # Update robot
            robot.update_kinematics(best_v, best_omega, dt)
            
            if step % 50 == 0:
                print(f"Step {step}: pos=({current_pose[0]:.2f}, {current_pose[1]:.2f}), "
                      f"goal_dist={distance_to_goal:.2f}m, v={best_v:.2f}, ω={best_omega:.2f}")
        
        except Exception as e:
            print(f"Planning error at step {step}: {e}")
            break
    
    return {
        'trajectory': robot_trajectory,
        'commands': control_commands,
        'predictions': predicted_trajectories,
        'obstacles': obstacles,
        'goal': goal,
        'start': start
    }


def compare_with_without_obstacles():
    """Compare navigation with and without obstacles."""
    start, goal, obstacles = create_test_scenario()
    
    # Create reference path without obstacles
    waypoints = [(start[0], start[1]), (10, 5), (goal[0], goal[1])]
    smoother = SplineSmoother()
    x_ref, y_ref = smoother.smooth(waypoints, num_points=100)
    reference_path = list(zip(x_ref, y_ref))
    
    # Simulate without obstacles (pure pursuit)
    robot1 = DifferentialDrive()
    robot1.set_state(start[0], start[1], start[2])
    controller = PurePursuitController(lookahead_distance=2.0)
    
    trajectory_no_obstacles = []
    dt = 0.1
    
    for _ in range(200):  # Limit steps
        pose = robot1.get_pose()
        trajectory_no_obstacles.append(pose)
        
        if controller.is_goal_reached(pose, goal, 0.5):
            break
        
        v_cmd, omega_cmd = controller.control(robot1, reference_path, 2.0)
        robot1.update_kinematics(v_cmd, omega_cmd, dt)
    
    # Simulate with obstacles (DWA)
    results_with_obstacles = simulate_obstacle_avoidance()
    
    return {
        'no_obstacles': trajectory_no_obstacles,
        'with_obstacles': results_with_obstacles,
        'reference_path': reference_path
    }


def plot_obstacle_avoidance_comparison():
    """Plot comparison of navigation with and without obstacles."""
    results = compare_with_without_obstacles()
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot without obstacles
    ax1.set_title('Navigation Without Obstacles (Pure Pursuit)')
    
    # Reference path
    ref_path = np.array(results['reference_path'])
    ax1.plot(ref_path[:, 0], ref_path[:, 1], 'k--', linewidth=2, alpha=0.7, label='Reference Path')
    
    # Robot trajectory
    traj_no_obs = np.array(results['no_obstacles'])
    ax1.plot(traj_no_obs[:, 0], traj_no_obs[:, 1], 'b-', linewidth=2, label='Robot Path')
    
    # Start and goal
    start_pos = results['with_obstacles']['start']
    goal_pos = results['with_obstacles']['goal']
    ax1.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='Start')
    ax1.plot(goal_pos[0], goal_pos[1], 'ro', markersize=10, label='Goal')
    
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.set_xlim(-1, 20)
    ax1.set_ylim(-1, 10)
    
    # Plot with obstacles
    ax2.set_title('Navigation With Obstacles (DWA)')
    
    # Obstacles
    obstacles = results['with_obstacles']['obstacles']
    for obs in obstacles:
        circle = patches.Circle((obs.x, obs.y), obs.radius, 
                              facecolor='red', alpha=0.5, edgecolor='red')
        ax2.add_patch(circle)
    
    # Robot trajectory
    traj_with_obs = np.array(results['with_obstacles']['trajectory'])
    ax2.plot(traj_with_obs[:, 0], traj_with_obs[:, 1], 'b-', linewidth=2, label='Robot Path')
    
    # Start and goal
    ax2.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='Start')
    ax2.plot(goal_pos[0], goal_pos[1], 'ro', markersize=10, label='Goal')
    
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    ax2.set_xlim(-1, 20)
    ax2.set_ylim(-1, 10)
    
    plt.tight_layout()
    plt.savefig('obstacle_avoidance_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def create_animated_obstacle_avoidance():
    """Create animated visualization of obstacle avoidance."""
    results = simulate_obstacle_avoidance()
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Draw obstacles
    obstacles = results['obstacles']
    for obs in obstacles:
        circle = patches.Circle((obs.x, obs.y), obs.radius, 
                              facecolor='red', alpha=0.5, edgecolor='red')
        ax.add_patch(circle)
    
    # Draw goal
    goal = results['goal']
    ax.plot(goal[0], goal[1], 'ro', markersize=12, label='Goal')
    
    # Initialize robot marker and trail
    robot_marker, = ax.plot([], [], 'bo', markersize=8, label='Robot')
    robot_trail, = ax.plot([], [], 'b-', alpha=0.5, linewidth=2)
    prediction_line, = ax.plot([], [], 'g--', alpha=0.7, linewidth=1, label='Prediction')
    
    ax.set_xlim(-1, 20)
    ax.set_ylim(-1, 10)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Dynamic Window Approach - Obstacle Avoidance')
    
    trajectory = results['trajectory']
    predictions = results['predictions']
    
    def animate(frame):
        if frame < len(trajectory):
            # Update robot position
            x, y, _ = trajectory[frame]
            robot_marker.set_data([x], [y])
            
            # Update trail
            if frame > 0:
                trail_x = [pos[0] for pos in trajectory[:frame+1]]
                trail_y = [pos[1] for pos in trajectory[:frame+1]]
                robot_trail.set_data(trail_x, trail_y)
            
            # Update prediction
            if frame < len(predictions) and predictions[frame]:
                pred_x = [pos[0] for pos in predictions[frame]]
                pred_y = [pos[1] for pos in predictions[frame]]
                prediction_line.set_data(pred_x, pred_y)
        
        return robot_marker, robot_trail, prediction_line
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, frames=len(trajectory),
                                 interval=100, blit=True, repeat=True)
    
    # Save animation
    try:
        anim.save('obstacle_avoidance_animation.gif', writer='pillow', fps=10)
        print("Animation saved as obstacle_avoidance_animation.gif")
    except Exception as e:
        print(f"Could not save animation: {e}")
    
    plt.show()


def analyze_dwa_performance():
    """Analyze DWA performance with different parameter settings."""
    start, goal, obstacles = create_test_scenario()
    
    # Test different parameter combinations
    parameter_sets = [
        {'goal_cost_gain': 1.0, 'obstacle_cost_gain': 1.0, 'speed_cost_gain': 0.1},
        {'goal_cost_gain': 2.0, 'obstacle_cost_gain': 1.0, 'speed_cost_gain': 0.1},
        {'goal_cost_gain': 1.0, 'obstacle_cost_gain': 2.0, 'speed_cost_gain': 0.1},
        {'goal_cost_gain': 1.0, 'obstacle_cost_gain': 1.0, 'speed_cost_gain': 0.3},
    ]
    
    results = {}
    
    print("DWA Parameter Analysis")
    print("=" * 40)
    
    for i, params in enumerate(parameter_sets):
        print(f"\nTesting parameter set {i+1}: {params}")
        
        # Initialize robot and DWA
        robot = DifferentialDrive()
        robot.set_state(start[0], start[1], start[2])
        
        dwa = DynamicWindowApproach(robot=robot, **params)
        
        # Simulate
        trajectory = []
        dt = 0.1
        max_steps = 300
        
        for step in range(max_steps):
            current_state = robot.get_state()
            current_pose = robot.get_pose()
            trajectory.append(current_pose)
            
            # Check goal
            distance_to_goal = np.sqrt((current_pose[0] - goal[0])**2 + 
                                     (current_pose[1] - goal[1])**2)
            
            if distance_to_goal < 0.5:
                break
            
            # Plan and execute
            try:
                v_cmd, omega_cmd = dwa.plan(current_state, goal, obstacles)
                robot.update_kinematics(v_cmd, omega_cmd, dt)
            except:
                break
        
        # Calculate metrics
        path_length = 0
        if len(trajectory) > 1:
            for j in range(1, len(trajectory)):
                dx = trajectory[j][0] - trajectory[j-1][0]
                dy = trajectory[j][1] - trajectory[j-1][1]
                path_length += np.sqrt(dx**2 + dy**2)
        
        completion_time = len(trajectory) * dt
        final_goal_distance = distance_to_goal
        
        results[f"Set {i+1}"] = {
            'path_length': path_length,
            'completion_time': completion_time,
            'final_distance': final_goal_distance,
            'success': final_goal_distance < 0.5
        }
        
        print(f"  Path length: {path_length:.2f}m")
        print(f"  Completion time: {completion_time:.1f}s")
        print(f"  Final distance to goal: {final_goal_distance:.3f}m")
        print(f"  Success: {final_goal_distance < 0.5}")
    
    return results


def main():
    """Main demonstration function."""
    print("Obstacle Avoidance Demo")
    print("======================")
    
    # Compare navigation with and without obstacles
    print("\n1. Comparing navigation with and without obstacles...")
    plot_obstacle_avoidance_comparison()
    
    # Create animated visualization
    print("\n2. Creating animated obstacle avoidance...")
    create_animated_obstacle_avoidance()
    
    # Analyze DWA performance
    print("\n3. Analyzing DWA parameter sensitivity...")
    analyze_dwa_performance()
    
    print("\nDemo completed! Check generated files:")
    print("  - obstacle_avoidance_comparison.png")
    print("  - obstacle_avoidance_animation.gif")


if __name__ == "__main__":
    main()
