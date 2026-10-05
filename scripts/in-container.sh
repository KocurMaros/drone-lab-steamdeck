#!/bin/bash
# Runs INSIDE the container: set up ROS 2 / Gazebo environment, then start a DroneLab app.
#   in-container.sh <python.module> [args...]     e.g. dronelab.gui.flight_app
#   in-container.sh --shell                         interactive shell with the same environment
#   in-container.sh --exec <command...>             run any command with the same environment
set -e
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY CYCLONEDDS_URI
export PYTHONPATH="/opt/dronelab${PYTHONPATH:+:$PYTHONPATH}"
export DRONELAB_VAR="${DRONELAB_VAR:-/opt/dronelab/var}"
export PATH="$HOME/.local/bin:$PATH"
mkdir -p "$DRONELAB_VAR/logs"
cd /opt/dronelab

if [ "${1:-}" = "--shell" ]; then
    echo "DroneLab container shell. Students' domain: ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    exec bash --rcfile <(echo "source ~/.bashrc 2>/dev/null; PS1='(dronelab) \w\$ '")
fi
if [ "${1:-}" = "--exec" ]; then shift; exec "$@"; fi
module="$1"; shift
exec > >(tee "$DRONELAB_VAR/logs/${module##*.}.log") 2>&1
exec python3 -m "$module" "$@"
