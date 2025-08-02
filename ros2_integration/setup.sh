#!/bin/bash

# ROS2 Path Smoothing TurtleBot Integration Setup Script

echo "==================================================="
echo "ROS2 Path Smoothing TurtleBot Setup"
echo "==================================================="

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please install ROS2 first:"
    echo "   sudo apt install ros-humble-desktop-full"
    exit 1
fi

echo "✅ ROS2 found"

# Check if in a ROS2 workspace
if [[ ! -f "package.xml" ]]; then
    echo "❌ Not in a ROS2 package directory"
    echo "Please run this script from the ros2_integration directory"
    echo "Or copy ros2_integration to your ROS2 workspace src folder"
    exit 1
fi

echo "✅ In ROS2 package directory"

# Make scripts executable
echo "🔧 Making scripts executable..."
chmod +x scripts/*.py
chmod +x launch/*.py

# Install TurtleBot3 packages if not present
echo "🔧 Checking TurtleBot3 packages..."
if ! ros2 pkg list | grep -q turtlebot3_gazebo; then
    echo "Installing TurtleBot3 packages..."
    sudo apt update
    sudo apt install -y ros-humble-turtlebot3*
    sudo apt install -y ros-humble-gazebo-ros-pkgs
    sudo apt install -y ros-humble-interactive-markers
else
    echo "✅ TurtleBot3 packages found"
fi

# Install Python dependencies
echo "🔧 Installing Python dependencies..."
pip install numpy scipy matplotlib opencv-python shapely

# Source ROS2 setup if in workspace
if [[ -f "../../install/setup.bash" ]]; then
    echo "🔧 Sourcing ROS2 workspace..."
    source ../../install/setup.bash
fi

echo ""
echo "==================================================="
echo "✅ Setup Complete!"
echo "==================================================="
echo ""
echo "Next steps:"
echo "1. Build the package:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select path_smoothing_turtlebot"
echo "   source install/setup.bash"
echo ""
echo "2. Set TurtleBot3 model:"
echo "   export TURTLEBOT3_MODEL=burger"
echo ""
echo "3. Launch simulation:"
echo "   ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py"
echo ""
echo "4. Or launch with options:"
echo "   ros2 launch path_smoothing_turtlebot turtlebot_path_smoothing.launch.py \\"
echo "       smoother_type:=bezier controller_type:=stanley"
echo ""
echo "See README.md for detailed usage instructions."
echo "==================================================="
