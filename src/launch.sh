SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../"   # Adjust if your scripts folder depth changes
source "$WORKSPACE_ROOT/install/setup.bash"
"./WORKSPACE_ROOT/src/3e8_ros2_foxglove/scripts/foxglove.sh" &
export DISPLAY=:0
xset s off
xset -dpms
xset s noblank
# Position Firefox window on the visible screen (remove the off-screen positioning)
flatpak run org.mozilla.firefox file:///home/ari/Desktop/demo.html --kiosk &
ros2 launch robot_bringup launch.py