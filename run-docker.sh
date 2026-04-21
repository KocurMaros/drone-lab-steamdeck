#!/bin/bash

# Steam Deck Docker/Podman Setup Script
# This scripts installs Podman (rootless, immutable-friendly alternative to Docker)
# since SteamOS is immutable, Podman is heavily preferred and natively supported flatpak CLI tool.

# 1. Aliasing podman -> docker for ease of use
echo "Setting up Podman as Docker replacement (preferred on SteamOS)..."

# Most SteamOS recent versions have podman installed or available without disabling rootfs.
# If not, let's just instruct to use distrobox/podman or standard pacman.

echo "Building the simulation environment container..."
# Build the Docker image from the Dockerfile
# We expect the user to have install.sh in their home directory, so copy it locally first:
if [ -f ~/install.sh ]; then
    cp ~/install.sh ./install.sh
else
    echo "Warning: ~/install.sh not found. Ensure it's in the current directory."
fi

# Use podman if available, fallback to docker
DOCKER_CMD="docker"
if command -v podman &> /dev/null; then
    DOCKER_CMD="podman"
fi

echo "Using container engine: $DOCKER_CMD"

# Build the image
$DOCKER_CMD build -t dronelab-sim-ubuntu2204 .

# Run the image with X11 forwarding for Gazebo/GUI
echo "Starting container with X11 forwarding for Gazebo & ROS..."
xhost +local:
$DOCKER_CMD run -it \
    --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/Desktop/LRS-FEI:/home/deck/LRS-FEI:rw" \
    --name="dronelab-sim" \
    dronelab-sim-ubuntu2204 /bin/bash
