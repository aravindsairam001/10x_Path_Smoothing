# TurtleBot Path Smoothing and Trajectory Control

This ROS2 package integrates the path smoothing and trajectory control algorithms with TurtleBot3 simulation in Gazebo and visualization in RViz.

## Features

- **Interactive Waypoint Setting**: Set waypoints using RViz interactive markers
- **Real-time Path Smoothing**: Choose between spline, Bezier, or B-spline smoothing
- **Multiple Controllers**: Pure Pursuit, Stanley, or PID trajectory following
- **Obstacle Avoidance**: Dynamic Window Approach (DWA) for collision avoidance
- **Full Simulation**: Complete TurtleBot3 Gazebo simulation environment
- **Visualization**: Comprehensive RViz visualization of paths, trajectories, and robot state

## Prerequisites

### ROS2 Installation
Make sure you have ROS2 Humble (or compatible) installed:
```bash
# Ubuntu 22.04
sudo apt update
sudo apt install ros-humble-desktop-full
```

### TurtleBot3 Packages
```bash
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Python Dependencies
```bash
pip install numpy scipy matplotlib opencv-python shapely
```

## Installation

1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Copy the ROS2 integration package:
```bash
cp -r /path/to/10x_Path_Smoothing/ros2_integration ~/ros2_ws/src/path_smoothing_turtlebot
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select path_smoothing_turtlebot
source install/setup.bash
```

## Usage

### 1. Complete Simulation (Recommended)

Launch the complete simulation with TurtleBot3 in Gazebo:

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch complete simulation
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py
```

**Optional parameters:**
```bash
# With different smoother and controller
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \
    smoother_type:=bezier \
    controller_type:=stanley

# With obstacle avoidance enabled
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \
    enable_obstacle_avoidance:=true \
    world:=path_smoothing_world
```

### 2. Path Smoothing Only

If you already have TurtleBot3 simulation running:

```bash
# Launch only the path smoothing nodes
ros2 launch path_smoothing_turtlebot path_smoothing_only.launch.py \
    smoother_type:=spline \
    controller_type:=pure_pursuit
```

### 3. Individual Nodes

You can also run individual nodes:

```bash
# Waypoint publisher
ros2 run path_smoothing_turtlebot waypoint_publisher_node.py

# Path smoothing
ros2 run path_smoothing_turtlebot path_smoothing_node.py \
    --ros-args -p smoother_type:=bspline

# Trajectory controller
ros2 run path_smoothing_turtlebot trajectory_controller_node.py \
    --ros-args -p controller_type:=pure_pursuit

# Obstacle avoidance
ros2 run path_smoothing_turtlebot obstacle_avoidance_node.py
```

## RViz Operation

Once the simulation is running, you can interact with the system through RViz:

### Setting Waypoints
1. **Interactive Markers**: Drag the red spheres to adjust waypoint positions
2. **Default Waypoints**: The system comes with a default set of waypoints
3. **Real-time Updates**: Path smoothing updates automatically as you move waypoints

### Path Visualization
- **Red Spheres**: Waypoints
- **Gray Line**: Connection between waypoints
- **Green Line**: Smoothed path
- **Blue Arrows**: Path direction indicators
- **Red Arrow**: Current robot pose

### Controller Operation
- **Automatic Following**: Robot automatically follows the smoothed path
- **Goal Tolerance**: Robot stops when within tolerance of the final waypoint
- **Real-time Control**: Control commands update at 10Hz

### Obstacle Avoidance (when enabled)
- **Set Goal**: Use "2D Nav Goal" tool in RViz to set a navigation goal
- **DWA Visualization**: Red trajectories show considered paths, green shows selected path
- **Dynamic Avoidance**: Robot dynamically avoids obstacles while moving toward goal

## ROS2 Topics

### Published Topics
- `/smooth_path` (nav_msgs/Path): Smoothed trajectory
- `/waypoints_markers` (visualization_msgs/MarkerArray): Waypoint visualization
- `/smooth_path_markers` (visualization_msgs/MarkerArray): Path visualization
- `/cmd_vel` (geometry_msgs/Twist): Robot velocity commands
- `/current_pose` (geometry_msgs/PoseStamped): Current robot pose
- `/dwa_trajectories` (visualization_msgs/MarkerArray): DWA trajectory visualization

### Subscribed Topics
- `/waypoints` (visualization_msgs/MarkerArray): Input waypoints
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/scan` (sensor_msgs/LaserScan): Laser scan data for obstacle avoidance
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): Navigation goal

## Parameters

### Path Smoothing Node
- `smoother_type`: Algorithm type [spline, bezier, bspline]
- `num_points`: Number of points in smoothed path (default: 100)
- `frame_id`: Reference frame (default: "map")

### Trajectory Controller Node
- `controller_type`: Controller type [pure_pursuit, stanley, pid]
- `lookahead_distance`: Pure pursuit lookahead distance (default: 1.0)
- `max_linear_vel`: Maximum linear velocity (default: 0.5 m/s)
- `max_angular_vel`: Maximum angular velocity (default: 1.0 rad/s)
- `goal_tolerance`: Goal reaching tolerance (default: 0.2 m)

### Obstacle Avoidance Node
- `goal_cost_gain`: Weight for goal cost (default: 1.0)
- `obstacle_cost_gain`: Weight for obstacle cost (default: 2.0)
- `speed_cost_gain`: Weight for speed cost (default: 0.1)
- `robot_radius`: Robot radius for collision checking (default: 0.2 m)

## Algorithm Comparison

You can test different combinations in real-time:

### Smoothing Algorithms
- **Spline**: Best for smooth curves, C² continuity
- **Bezier**: Good control over curve shape, local control
- **B-spline**: Flexible, good for complex paths

### Control Algorithms
- **Pure Pursuit**: Simple, robust, good for most applications
- **Stanley**: Better lateral control, good for tight tracking
- **PID**: Simple feedback control, may need tuning

## Troubleshooting

### Common Issues

1. **"No module named 'path_smoothing'"**
   - Make sure the Python path is correctly set in the node scripts
   - Verify the src directory structure is correct

2. **TurtleBot3 model not found**
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

3. **Gazebo doesn't start**
   ```bash
   # Check if Gazebo is already running
   killall gazebo gzserver gzclient
   ```

4. **RViz configuration not found**
   ```bash
   # Launch with default RViz config
   ros2 run rviz2 rviz2
   ```

### Performance Tips

1. **Reduce path points** for better performance:
   ```bash
   ros2 param set /path_smoothing num_points 50
   ```

2. **Adjust control frequency** if needed:
   ```bash
   ros2 param set /trajectory_controller control_rate 5.0
   ```

3. **Use simpler smoother** for complex paths:
   ```bash
   ros2 param set /path_smoothing smoother_type spline
   ```

## Demo Scenarios

### Scenario 1: Basic Path Following
```bash
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \
    smoother_type:=spline controller_type:=pure_pursuit
```

### Scenario 2: Advanced Control
```bash
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \
    smoother_type:=bspline controller_type:=stanley
```

### Scenario 3: Obstacle Avoidance
```bash
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \
    enable_obstacle_avoidance:=true world:=path_smoothing_world
```

## Development

### Adding New Algorithms

1. **New Smoother**: Add to `src/path_smoothing/`
2. **New Controller**: Add to `src/trajectory_control/`
3. **Update Nodes**: Modify the corresponding ROS2 nodes
4. **Test**: Use the existing test framework

### Custom Worlds

Create custom Gazebo worlds in the `worlds/` directory and reference them in launch files.

## Performance Metrics

The system provides real-time performance monitoring:
- Path smoothing computation time
- Control loop frequency
- Trajectory tracking error
- Goal reaching success rate

Monitor through ROS2 topics and node logs for optimization.

---

This integration brings the powerful path smoothing and trajectory control algorithms into a complete robotics simulation environment, making it perfect for research, education, and development of autonomous navigation systems.
