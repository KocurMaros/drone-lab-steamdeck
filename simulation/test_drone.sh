#!/bin/bash
echo "Setting mode to GUIDED..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
sleep 6

echo "Arming drone..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
sleep 2

echo "Taking off..."
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0, yaw: 90, altitude: 2}"
echo "Takeoff command sent!"
