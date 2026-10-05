#!/bin/bash
# Wrapper to start the container if not running, then launch the simulation components

# Start container logic
if ! docker ps -a | grep -q "dronelab-sim" || ! docker ps | grep -q "dronelab-sim"; then
    echo "Starting container..."
    konsole -e /home/deck/Desktop/dronelab/scripts/run-docker.sh &
    echo "Waiting for container to initialize..."
    sleep 15
fi

# Give Docker a moment to fully register TTY and X11
sleep 2

# Launch Gazebo with the new Bayland world
# We disabled online models (GAZEBO_MODEL_DATABASE_URI="") so it doesn't hang trying to download on launch.
# We explicitly define the Gazebo model paths so it doesn't fail trying to fetch sun/ground_plane from the web
konsole --noclose -e /home/deck/Desktop/dronelab/scripts/exec-in-docker.sh -- bash -ic "cd /home/deck/LRS-FEI && source /home/deck/LRS-FEI/gazebo_setup.bash && export GAZEBO_MODEL_DATABASE_URI=\"\" && export GAZEBO_MODEL_PATH=/home/deck/LRS-FEI/models:/usr/share/gazebo-11/models:/home/deck/.gazebo/models:\$GAZEBO_MODEL_PATH && gazebo /home/deck/LRS-FEI/worlds/bayland.world; exec bash" &

sleep 5

# Launch ArduPilot SITL with the iris_camera model
konsole --noclose -e /home/deck/Desktop/dronelab/scripts/exec-in-docker.sh -- bash -ic "cd /home/deck/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console; exec bash" &

echo "SITL and Gazebo are starting..."