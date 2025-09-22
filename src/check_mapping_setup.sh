#!/usr/bin/env bash

# Authors	  : Ari, Ikshwak
# Date		  : August 1 2025
# Description : 3E8 Robot Mapping Setup Checker
#               This script verifies that all components are properly installed and configured

#!/bin/bash



echo "Checking 3E8 Robot Mapping Setup..."
echo "=================================="

# Change to the ROS2 workspace directory
  # Use the following directory if on the pc (used by Ikshwak)
cd /home/ikshwak/3e8_v1/3e8_ros2
  # Use the following directory if on the actual robot.
# cd /home/ari/3e8_v1/3e8_ros2

# Source the ROS2 environment
source install/setup.bash

echo ""
echo "1. Checking ROS2 packages..."

# Check for required packages
packages=("foxglove_bridge" "slam_toolbox" "nav2_map_server" "robot_state_publisher" "tf2_ros")
all_packages_ok=true

for package in "${packages[@]}"; do
    if ros2 pkg list | grep -q "$package"; then
        echo "  ✓ $package - installed"
    else
        echo "  ✗ $package - missing"
        all_packages_ok=false
    fi
done

echo ""
echo "2. Checking configuration files..."

# Check for configuration files
config_files=(
    "src/3e8_ros2_bringup/config/slam_toolbox_params.yaml"
    "src/3e8_ros2_bringup/config/mapping.rviz"
    "src/3e8_ros2_bringup/urdf/robot.urdf"
    "foxglove_mapping_layout.json"
)

all_configs_ok=true
for config in "${config_files[@]}"; do
    if [ -f "$config" ]; then
        echo "  ✓ $config - found"
    else
        echo "  ✗ $config - missing"
        all_configs_ok=false
    fi
done

echo ""
echo "3. Checking launch files..."

# Check for launch files
launch_files=(
    "src/3e8_ros2_bringup/launch/mapping_launch.py"
    "test_mapping.sh"
    "foxglove.sh"
)

all_launch_ok=true
for launch in "${launch_files[@]}"; do
    if [ -f "$launch" ]; then
        echo "  ✓ $launch - found"
    else
        echo "  ✗ $launch - missing"
        all_launch_ok=false
    fi
done

echo ""
echo "4. Checking executable permissions..."

# Check permissions
if [ -x "test_mapping.sh" ]; then
    echo "  ✓ test_mapping.sh - executable"
else
    echo "  ✗ test_mapping.sh - not executable"
    all_launch_ok=false
fi

if [ -x "foxglove.sh" ]; then
    echo "  ✓ foxglove.sh - executable"
else
    echo "  ✗ foxglove.sh - not executable"
    all_launch_ok=false
fi

echo ""
echo "5. Checking camera access..."

# Check if camera is accessible (this might fail if camera is not connected)
if ros2 topic list | grep -q "/camera"; then
    echo "  ✓ Camera topics detected"
else
    echo "  ⚠ Camera topics not detected (camera may not be connected)"
fi

echo ""
echo "=================================="
echo "Setup Check Complete"
echo "=================================="

if [ "$all_packages_ok" = true ] && [ "$all_configs_ok" = true ] && [ "$all_launch_ok" = true ]; then
    echo "✓ All components are properly installed and configured!"
    echo ""
    echo "To start mapping:"
    echo "  ./test_mapping.sh"
    echo ""
    echo "To connect Foxglove:"
    echo "  1. Open https://studio.foxglove.dev/"
    echo "  2. Connect to ws://localhost:9090"
    echo "  3. Import layout: foxglove_mapping_layout.json"
else
    echo "✗ Some components are missing or misconfigured."
    echo "Please check the errors above and fix them before proceeding."
fi
