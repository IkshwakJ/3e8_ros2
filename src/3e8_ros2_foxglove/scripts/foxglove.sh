#!/usr/bin/env bash
set -Eeo pipefail   # keep strict, but omit -u (nounset)

echo "Starting Foxglove Bridge for mapping visualization..."
echo "Connect to: ws://localhost:9090"
echo "Press Ctrl+C to stop"

# --- Source ROS environments safely (some vars may be unset) ---
if [ -f /opt/ros/humble/setup.bash ]; then
  # ensure var exists even if ros setup references it
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
  source /opt/ros/humble/setup.bash
fi

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../../.."   # Adjust if your scripts folder depth changes
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
fi

# --- Run bridge ---
exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p port:=9090 \
  -p send_buffer_limit:=524288000 \
  -p drop_messages_when_full:=true \
  -p queue_size:=5 \
  -p use_compression:=true
