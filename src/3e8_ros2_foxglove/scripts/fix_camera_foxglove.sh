#!/bin/bash

#!/bin/bash
# Authors     : Ari, Ikshwak
# Purpose     : Fix Camera Live Feed in Foxglove from within a ROS2 package
# Usage       : ./scripts/fix_camera_foxglove.sh (after sourcing workspace)

# Fix Camera Live Feed in Foxglove
echo "Fixing camera live feed in Foxglove..."

# --- Source ROS2 workspace environment ---
# Use the absolute path to your ROS2 workspace install/setup.bash
# This ensures the script works regardless of where you call it from
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../../.."   # Adjust if your scripts folder depth changes
source "$WORKSPACE_ROOT/install/setup.bash"

# Kill existing foxglove bridge
echo "Stopping existing Foxglove bridge..."
pkill -f foxglove_bridge
sleep 2

# Check if camera topics exist
echo "Checking camera topics..."
ros2 topic list | grep -i camera

# Check camera data rate
echo "Checking camera data rate..."
timeout 5 ros2 topic hz /camera/color/image_raw || echo "No data from camera"

# Restart Foxglove bridge with compression
echo "Starting Foxglove bridge with image compression..."
ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p port:=9090 \
  -p send_buffer_limit:=524288000 \
  -p drop_messages_when_full:=true \
  -p queue_size:=5 \
  -p use_compression:=true &

echo "Foxglove bridge started!"
echo "Connect to: ws://localhost:9090"
echo "Add Image panel with topic: /camera/color/image_raw"
