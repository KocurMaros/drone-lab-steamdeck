#!/bin/bash

echo "Building the simulation environment container in Docker..."

# Fallback to docker if podman is broken, or use podman if available
DOCKER_CMD="docker"
if command -v podman &> /dev/null; then
    DOCKER_CMD="podman"
fi

echo "Using container engine: $DOCKER_CMD"

# Build the layered image directly
$DOCKER_CMD build -t dronelab-sim-ubuntu2204 .

# Run the image with X11 forwarding for Gazebo/GUI
echo "Starting container with X11 forwarding for Gazebo & ROS..."
xhost +local: || true

$DOCKER_CMD run -it \
    --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/Projects:/home/deck/Projects:rw" \
    --volume="$HOME/LRS-FEI:/home/deck/LRS-FEI:rw" \
    --name="dronelab-sim" \
    dronelab-sim-ubuntu2204 /bin/bash
