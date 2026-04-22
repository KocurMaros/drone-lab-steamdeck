#!/bin/bash
# Helper script to route desktop app commands to the running Docker/Podman container

DOCKER_CMD="docker"
if command -v podman &> /dev/null; then
    DOCKER_CMD="podman"
fi

if ! $DOCKER_CMD ps | grep -q "dronelab-sim"; then
    echo "ERROR: The 'dronelab-sim' container is not running!"
    echo "Please open run-docker.sh and leave it running in the background first."
    echo "Press Enter to exit..."
    read -r
    exit 1
fi

exec $DOCKER_CMD exec -it dronelab-sim "$@"
