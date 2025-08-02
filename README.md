# Path Smoothing and Trajectory Control in 2D Space

This project implements a path smoothing algorithm and trajectory tracking controller for a differential drive robot navigating through a series of 2D waypoints.

## Features

1. **Path Smoothing**: Convert discrete waypoints into smooth, continuous trajectories using:
   - Cubic spline interpolation
   - B-spline smoothing
   - Bezier curve smoothing

2. **Trajectory Tracking**: Control differential drive robot to follow the smooth trajectory using:
   - Pure pursuit controller
   - Stanley controller
   - PID-based controller

3. **Obstacle Avoidance** (Extra Credit): Dynamic path adjustment to avoid obstacles

## Project Structure

```
├── src/
│   ├── path_smoothing/
│   │   ├── __init__.py
│   │   ├── spline_smoother.py
│   │   ├── bezier_smoother.py
│   │   └── bspline_smoother.py
│   ├── trajectory_control/
│   │   ├── __init__.py
│   │   ├── differential_drive.py
│   │   ├── pure_pursuit.py
│   │   ├── stanley_controller.py
│   │   └── pid_controller.py
│   ├── obstacle_avoidance/
│   │   ├── __init__.py
│   │   └── dynamic_window.py
│   └── utils/
│       ├── __init__.py
│       └── geometry.py
├── examples/
│   ├── basic_smoothing_demo.py
│   ├── trajectory_following_demo.py
│   └── obstacle_avoidance_demo.py
├── tests/
│   ├── test_path_smoothing.py
│   └── test_trajectory_control.py
├── ros2_integration/               # NEW: ROS2 integration
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── scripts/
│   │   ├── path_smoothing_node.py
│   │   ├── trajectory_controller_node.py
│   │   ├── waypoint_publisher_node.py
│   │   └── obstacle_avoidance_node.py
│   ├── launch/
│   │   ├── turtlebot_path_smoothing.launch.py
│   │   └── path_smoothing_only.launch.py
│   ├── config/
│   │   └── path_smoothing.rviz
│   ├── worlds/
│   │   └── path_smoothing_world.world
│   └── README.md
├── requirements.txt
└── main.py
```

## Installation

```bash
pip install -r requirements.txt
```

## Usage

### Basic Path Smoothing
```python
from src.path_smoothing import SplineSmoother

waypoints = [(0, 0), (5, 2), (10, 1), (15, 3)]
smoother = SplineSmoother()
smooth_path = smoother.smooth(waypoints, num_points=100)
```

### Trajectory Following
```python
from src.trajectory_control import PurePursuitController
from src.trajectory_control import DifferentialDrive

robot = DifferentialDrive()
controller = PurePursuitController(lookahead_distance=2.0)
control_commands = controller.control(robot.pose, smooth_path)
```

### ROS2 Integration with TurtleBot

For complete simulation with TurtleBot3 in Gazebo and RViz:

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch complete simulation
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py

# With different algorithms
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \
    smoother_type:=bezier controller_type:=stanley

# With obstacle avoidance
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \
    enable_obstacle_avoidance:=true
```

See `ros2_integration/README.md` for detailed ROS2 setup and usage instructions.

## Running Examples

### Standalone Python Examples
```bash
python examples/basic_smoothing_demo.py
python examples/trajectory_following_demo.py
python examples/obstacle_avoidance_demo.py
```

### Main Application
```bash
# Run basic demo
python3 main.py

# Compare all algorithms
python3 main.py --compare

# Test different configurations
python3 main.py --pattern s_curve --smoother bspline --controller pure_pursuit
```

### ROS2 TurtleBot Simulation

Complete robotics simulation with TurtleBot3:

1. **Install ROS2 and TurtleBot3 packages**:
```bash
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros-pkgs
```

2. **Build the ROS2 package**:
```bash
cd ~/ros2_ws/src
cp -r /path/to/10x_Path_Smoothing/ros2_integration path_smoothing_turtlebot
cd ~/ros2_ws
colcon build --packages-select path_smoothing_turtlebot
source install/setup.bash
```

3. **Launch simulation**:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py
```

4. **Interact with RViz**:
   - Drag red spheres to set waypoints
   - Watch real-time path smoothing and robot following
   - Use "2D Nav Goal" for obstacle avoidance mode

See `ros2_integration/README.md` for detailed instructions.
