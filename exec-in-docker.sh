#!/bin/bash
# Helper script to route desktop app commands to the running Docker/Podman container

export PATH="/home/deck/.local/bin:/usr/local/bin:/usr/bin:/bin:$PATH"

DOCKER_CMD="docker"
if command -v podman &> /dev/null; then
    DOCKER_CMD="podman"
elif [ -x "/home/deck/.local/bin/podman" ]; then
    DOCKER_CMD="/home/deck/.local/bin/podman"
elif [ -x "/usr/bin/podman" ]; then
    DOCKER_CMD="/usr/bin/podman"
fi

if ! $DOCKER_CMD ps -a | grep -q "dronelab-sim"; then
    echo "ERROR: The 'dronelab-sim' container is not running or found by $DOCKER_CMD!"
    echo "Please open run-docker.sh and leave it running in the background first."
    read -r
    exit 1
fi

if [ "$1" = "--" ]; then
    shift
fi

exec $DOCKER_CMD exec -it dronelab-sim "$@"
