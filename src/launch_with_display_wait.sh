#!/bin/bash

# Enhanced launch script for boot scenarios
# Wait for display to be ready

echo "Waiting for display to be ready..."

# Wait for X11 to be available
while [ -z "$DISPLAY" ]; do
    export DISPLAY=:0
    sleep 1
done

# Wait for X server to respond
timeout=30
count=0
while ! xset q >/dev/null 2>&1; do
    sleep 1
    count=$((count + 1))
    if [ $count -ge $timeout ]; then
        echo "Timeout waiting for X server"
        exit 1
    fi
done

echo "Display ready, starting robot services..."

# Change to the correct directory
cd ../

# Source ROS2 environment - need both global ROS2 and local workspace
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Set additional environment variables for ROS2
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Verify ROS2 is working
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 command not found after sourcing"
    exit 1
fi

echo "ROS2 environment ready (distro: $ROS_DISTRO)"

# Start foxglove bridge
./3e8_ros2_foxglove/scripts/foxglove.sh &

# Configure display settings
export DISPLAY=:0
xset s off
xset -dpms
xset s noblank

# Wait a bit for services to initialize
sleep 3

# Launch Firefox with robot interface
echo "Starting robot interface..."

# TODO
# Change file location in the following command to match the position of
# the actual file.
flatpak run org.mozilla.firefox file:///home/ari/Desktop/demo.html --kiosk &

# Verify the workspace is built
if [ ! -d "install/3e8_ros2_bringup" ]; then
    echo "ERROR: robot_bringup package not found in install directory"
    echo "The workspace may not be built. Try running: colcon build"
    exit 1
fi

# Start ROS2 robot launch
echo "Starting ROS2 robot launch..."
echo "Launching: ros2 launch 3e8_ros2_bringup launch.py"

# Add logging for debugging
exec > >(tee -a /tmp/robot_launch.log) 2>&1

# Launch with error handling
if ! ros2 launch 3e8_ros2_bringup launch.py; then
    echo "ERROR: Failed to launch robot_bringup launch.py"
    echo "Check /tmp/robot_launch.log for details"
    exit 1
fi