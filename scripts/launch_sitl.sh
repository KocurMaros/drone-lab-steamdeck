#!/bin/bash
/home/deck/Desktop/dronelab/scripts/exec-in-docker.sh -- bash -ic 'read -p "Enter Drone SYS_ID (default 1): " sysid; sysid=${sysid:-1}; echo "Launching SITL with SYS_ID=$sysid"; cd /home/deck/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid=$sysid --console; exec bash'
