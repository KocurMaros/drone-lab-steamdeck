#!/bin/bash
# Test
echo "--- EXEC LOG ---" >> /tmp/exec-docker.log
echo "Environment:" >> /tmp/exec-docker.log
env >> /tmp/exec-docker.log
export PATH="/home/deck/.local/bin:/usr/local/bin:/usr/bin:/bin:$PATH"

DOCKER_CMD="docker"
if command -v podman &> /dev/null; then
    DOCKER_CMD="podman"
elif [ -x "/home/deck/.local/bin/podman" ]; then
    DOCKER_CMD="/home/deck/.local/bin/podman"
fi

$DOCKER_CMD ps -a >> /tmp/exec-docker.log 2>&1
echo "Checking grep:" >> /tmp/exec-docker.log
if $DOCKER_CMD ps -a | grep -q "dronelab-sim"; then
    echo "Found!" >> /tmp/exec-docker.log
else
    echo "Not found!" >> /tmp/exec-docker.log
fi

if [ "$1" = "--" ]; then
    shift
fi

exec $DOCKER_CMD exec -it dronelab-sim "$@"
