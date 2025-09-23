ros2 daemon stop
ros2 daemon start
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../"   # Adjust if your scripts folder depth changes
source "$WORKSPACE_ROOT/install/setup.bash"
ros2 launch robot_bringup mapping_launch.py