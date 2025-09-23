#!/bin/bash
# Save the current RTAB-Map occupancy grid to standard map files
echo "Saving map from /map topic..."
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../"   # Adjust if your scripts folder depth changes
source "$WORKSPACE_ROOT/install/setup.bash"
ros2 run nav2_map_server map_saver_cli -f maps/rtabmap_map
echo "Map saved to maps/rtabmap_map.pgm and maps/rtabmap_map.yaml"
