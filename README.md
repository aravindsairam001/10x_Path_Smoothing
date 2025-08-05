# 10x_Path_Smoothing

A comprehensive ROS2-based project for advanced path smoothing and trajectory following with TurtleBot3. This repository implements a complete autonomous navigation pipeline featuring cubic spline path smoothing, Stanley controller for precise path following, and dynamic obstacle avoidance capabilities.

View Full Working Simulation - [Click Here](https://drive.google.com/file/d/1LspiAaazAiD-Kn80T5TisoxVWLdXRBXj/view)

## Overview

This project demonstrates state-of-the-art path planning and control algorithms for autonomous mobile robots. The system combines mathematical path smoothing techniques with robust control theory to achieve smooth, efficient robot navigation. Key features include cubic spline interpolation for smooth trajectories, Stanley controller for accurate path following, and comprehensive visualization tools for analysis and debugging.

![Path Smoothing](trajectory_following_animation.gif)

![Obstacle_Avoidance](obstacle_avoidance_animation.gif)

## Features

- **Advanced Path Smoothing**: Cubic spline interpolation for C² continuous trajectories
- **Robust Path Following**: Stanley controller with cross-track error correction
- **Dynamic Obstacle Avoidance**: Real-time obstacle detection and path modification
- **Comprehensive Visualization**: RViz integration with color-coded waypoints and path visualization
- **ROS2 Native**: Built for ROS2 Humble with modern robotics architecture
- **TurtleBot3 Integration**: Optimized for TurtleBot3 Burger/Waffle platforms
- **Real-time Performance**: 15Hz control loop with efficient algorithms
- **Extensible Architecture**: Modular design for easy customization and extension

## Repository Structure

This repository contains multiple implementations and demonstration scripts:

### Core ROS2 Package (`turtlebot_ws/`)
- **TurtleBot3 Path Smoothing Node**: Complete ROS2 implementation
- **Cubic Spline Smoother**: Mathematical path smoothing algorithms
- **Stanley Controller**: Advanced path following controller
- **Launch Files**: Automated simulation startup

### Standalone Examples (`examples/`)
- **Basic Smoothing Demo**: Simple path smoothing visualization
- **Obstacle Avoidance Demo**: Dynamic obstacle avoidance demonstration
- **Trajectory Following Demo**: Complete trajectory following example

### Algorithm Implementations (`src/`)
- **Path Smoothing**: Bézier, B-spline, and cubic spline implementations
- **Trajectory Control**: Pure pursuit, Stanley, and PID controllers
- **Obstacle Avoidance**: Dynamic window approach implementation
- **Utility Functions**: Geometry and mathematical helper functions

### Testing Suite (`tests/`)
- **Unit Tests**: Comprehensive algorithm validation
- **Integration Tests**: Full system testing

## Getting Started

### Prerequisites

- **ROS2 Humble** (or compatible version)
- **Ubuntu 22.04** (recommended)
- **TurtleBot3 packages** for ROS2
- **Python 3.8+** with required dependencies
- **Gazebo** for simulation
- **RViz2** for visualization

### System Dependencies

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop-full

# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Install additional ROS2 packages
sudo apt install ros-humble-gazebo-* ros-humble-navigation2 ros-humble-nav2-bringup
```

### Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/aravindsairam001/10x_Path_Smoothing.git
    cd 10x_Path_Smoothing
    ```

2. **Install Python dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3. **Build the ROS2 workspace**:
    ```bash
    cd turtlebot_ws
    colcon build --packages-select turtlebot_path_smoothing
    source install/setup.bash
    ```

4. **Set TurtleBot3 model** (required for simulation):
    ```bash
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    source ~/.bashrc
    ```

## Running the System

### Option 1: Complete ROS2 Simulation (Recommended)

Run the full TurtleBot3 simulation with path smoothing:

```bash
cd turtlebot_ws
source install/setup.bash

# Launch complete simulation (Gazebo + RViz + Path Smoothing)
ros2 launch turtlebot_path_smoothing complete_simulation.launch.py
```

This will:
- Start Gazebo simulation with TurtleBot3
- Launch RViz2 with pre-configured visualization
- Initialize the path smoothing node with demo trajectory
- Begin autonomous navigation with real-time visualization

### Option 2: Individual Components

Run components separately for development and testing:

```bash
# Terminal 1: Start TurtleBot3 simulation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Start path smoothing node
cd turtlebot_ws && source install/setup.bash
ros2 run turtlebot_path_smoothing turtlebot_path_node

# Terminal 3: Launch RViz for visualization
ros2 launch turtlebot3_bringup rviz2.launch.py
```

### Option 3: Standalone Examples

Run individual algorithm demonstrations:

```bash
# Basic path smoothing visualization
python examples/basic_smoothing_demo.py

# Obstacle avoidance demonstration
python examples/obstacle_avoidance_demo.py

# Complete trajectory following example
python examples/trajectory_following_demo.py
```

### Option 4: Algorithm Testing

Run the main algorithm comparison script:

```bash
python main.py
```

This generates comparison plots and analysis of different smoothing algorithms.

## Project Structure

```
10x_Path_Smoothing/
├── turtlebot_ws/                           # ROS2 Workspace
│   ├── src/turtlebot_path_smoothing/      # Main ROS2 Package
│   │   ├── turtlebot_path_smoothing/      # Python Package
│   │   │   ├── turtlebot_path_node.py     # Main ROS2 Node
│   │   │   ├── cubic_spline_smoother.py   # Spline Implementation
│   │   │   └── stanley_controller.py      # Path Following Controller
│   │   ├── launch/                        # Launch Files
│   │   │   └── complete_simulation.launch.py
│   │   ├── package.xml                    # ROS2 Package Definition
│   │   └── setup.py                       # Python Package Setup
│   ├── build/                             # Build Artifacts
│   ├── install/                           # Installation Files
│   └── log/                               # Build Logs
├── examples/                              # Standalone Examples
│   ├── basic_smoothing_demo.py           # Simple smoothing demo
│   ├── obstacle_avoidance_demo.py        # Obstacle avoidance demo
│   └── trajectory_following_demo.py      # Complete trajectory demo
├── src/                                   # Algorithm Implementations
│   ├── path_smoothing/                    # Smoothing Algorithms
│   │   ├── bezier_smoother.py            # Bézier curve smoothing
│   │   ├── bspline_smoother.py           # B-spline smoothing
│   │   └── spline_smoother.py            # Cubic spline smoothing
│   ├── trajectory_control/               # Control Algorithms
│   │   ├── pure_pursuit.py               # Pure pursuit controller
│   │   ├── stanley_controller.py         # Stanley controller
│   │   └── pid_controller.py             # PID controller
│   ├── obstacle_avoidance/               # Obstacle Avoidance
│   │   └── dynamic_window.py             # Dynamic window approach
│   └── utils/                            # Utility Functions
│       └── geometry.py                   # Geometric calculations
├── tests/                                # Test Suite
│   ├── test_path_smoothing.py           # Smoothing algorithm tests
│   └── test_trajectory_control.py       # Control algorithm tests
├── main.py                               # Algorithm comparison script
├── requirements.txt                      # Python dependencies
├── TECHNICAL_DOCUMENTATION.md           # Detailed technical docs
└── README.md                            # This file
```

## Key Components

### 1. Path Smoothing Algorithms
- **Cubic Spline**: C² continuous interpolation for smooth trajectories
- **Bézier Curves**: Parametric curve generation with control points
- **B-Splines**: Non-uniform rational basis splines for complex paths

### 2. Path Following Controllers
- **Stanley Controller**: Cross-track error correction with heading control
- **Pure Pursuit**: Geometric path following with lookahead distance
- **PID Controller**: Classical control for trajectory tracking

### 3. Obstacle Avoidance
- **Dynamic Window Approach**: Real-time velocity space planning
- **Collision Detection**: Efficient obstacle checking algorithms
- **Path Replanning**: Dynamic trajectory modification

### 4. Visualization and Analysis
- **RViz Integration**: Real-time path and robot state visualization
- **Performance Metrics**: Quantitative analysis of smoothing quality
- **Comparative Analysis**: Side-by-side algorithm comparison

## Usage Examples

### Interactive Goal Setting
Once the simulation is running, you can:
1. Use RViz2's "2D Nav Goal" tool to set custom waypoints
2. Watch the robot plan and execute smoothed trajectories
3. Observe real-time visualization of path following performance

### Parameter Tuning
Modify key parameters in the launch file or node:
```python
# Control parameters
goal_tolerance = 0.25           # Waypoint reaching tolerance
max_linear_velocity = 0.4       # Maximum robot speed
control_frequency = 15.0        # Control loop frequency

# Smoothing parameters
spline_resolution = 0.1         # Path point density
stanley_gain = 0.5              # Cross-track error gain
```

### Custom Waypoint Sequences
Modify `initialize_demo_path()` in `turtlebot_path_node.py` to test different trajectory patterns:
- S-curves for smooth acceleration/deceleration
- Figure-8 patterns for complex maneuvering
- Circular paths for constant curvature testing

## Performance Analysis

The system generates comprehensive performance visualizations:
- **Path smoothing comparison plots**: Visual comparison of different algorithms
- **Curvature analysis**: Quantitative smoothness metrics
- **Real-time trajectory tracking**: Live performance monitoring

View generated analysis plots:
- `path_smoothing_comparison.png`: Algorithm comparison
- `curvature_analysis.png`: Smoothness metrics
- `trajectory_following_comparison.png`: Control performance

## Troubleshooting

### Common Issues

1. **TurtleBot3 model not set**:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

2. **ROS2 workspace not sourced**:
   ```bash
   cd turtlebot_ws && source install/setup.bash
   ```

3. **Missing dependencies**:
   ```bash
   sudo apt install ros-humble-turtlebot3-gazebo
   pip install -r requirements.txt
   ```

4. **Gazebo doesn't start**:
   ```bash
   # Check if Gazebo is already running
   killall gzserver gzclient
   ```

### System Requirements
- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: Dedicated graphics card for optimal Gazebo performance
- **Storage**: 2GB free space for installation and logs

## Technical Documentation

For detailed technical information, including:
- Algorithm implementation details
- Architectural decisions and design choices
- Extension to real robots
- Obstacle avoidance strategies

See: [`TECHNICAL_DOCUMENTATION.md`](TECHNICAL_DOCUMENTATION.md)

## Testing

Run the comprehensive test suite:

```bash
# Unit tests for individual algorithms
python -m pytest tests/test_path_smoothing.py
python -m pytest tests/test_trajectory_control.py

# Integration tests (requires ROS2 environment)
cd turtlebot_ws && source install/setup.bash
colcon test --packages-select turtlebot_path_smoothing
```

## Contributing

We welcome contributions! Here's how you can help:

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Commit your changes**: `git commit -m 'Add amazing feature'`
4. **Push to branch**: `git push origin feature/amazing-feature`
5. **Open a Pull Request**

### Development Guidelines
- Follow PEP 8 style guidelines for Python code
- Add unit tests for new algorithms
- Update documentation for new features
- Test with both simulation and real robot (if available)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **ROS2 Community**: For the excellent robotics framework
- **TurtleBot3 Team**: For the outstanding robot platform
- **Open Source Robotics**: For inspiration and reference implementations

## Author

**Aravind SaiRam**
- GitHub: [@aravindsairam001](https://github.com/aravindsairam001)
- Email: Contact through GitHub issues

---

## Quick Start Summary

```bash
# 1. Clone and setup
git clone https://github.com/aravindsairam001/10x_Path_Smoothing.git
cd 10x_Path_Smoothing
pip install -r requirements.txt

# 2. Build ROS2 package
cd turtlebot_ws
colcon build --packages-select turtlebot_path_smoothing
source install/setup.bash

# 3. Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# 4. Launch complete simulation
ros2 launch turtlebot_path_smoothing complete_simulation.launch.py
```

**Watch the magic happen!** 🚀 The TurtleBot will autonomously navigate using advanced path smoothing and control algorithms.
