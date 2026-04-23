# DroneLab Simulation Guide

This guide explains how to launch the simulated DroneLab environment using Gazebo, ArduPilot SITL (Software In The Loop), and MAVROS. This simulation allows you to test ROS2 commands and flight logic before deploying them to the real drones.

## 🚀 Launching the Simulation

To start the full simulation environment, launch the following shortcuts from your desktop in the given order.

*(Note: These shortcuts use `konsole` and `distrobox` to run processes inside the DroneLab Docker container.)*

### 1. Launch Gazebo Environment
Double-click **`Gazebo Environment`** (`gazebo_env.desktop`).
- This opens the 3D physics simulator.
- It loads the LRS-FEI world (`fei_lrs_gazebo.world`).
- Wait until the Gazebo GUI is fully loaded and you can see the simulated drone.

### 2. Launch ArduPilot SITL
Double-click **`ArduPilot SITL`** (`ardupilot_sitl.desktop`).
- This runs the ArduCopter firmware in simulation mode.
- It connects to the Gazebo physics engine via the `gazebo-iris` model.
- You will see a MAVProxy console pop up. Wait until it says `EKF3 is using GPS` and `Ready to FLY`.

### 3. Launch MAVROS
Double-click **`LRS-FEI MAVROS`** (`lrs_fei_mavros.desktop`).
- This bridges the simulated ArduPilot (MAVLink) to ROS2 topics.
- MAVROS will connect to the SITL drone at `udp://127.0.0.1:14550@14555`.

---

## 🕹️ Testing the Drone

Once all three windows are open and running, you can test if the drone responds to ROS2 commands.

Double-click **`test_drone`** (`test_drone.desktop`) or run the script manually:
```bash
./test_drone.sh
```

**What `test_drone.sh` does:**
1. Sets the drone mode to `GUIDED`.
2. Arms the motors.
3. Issues a takeoff command to reach an altitude of 2 meters.

You should see the drone take off in the Gazebo 3D view.

---

## 📡 Interacting via ROS2

In the simulation, the default MAVROS namespace is typically `/mavros` (unlike the real drone setup which uses domain bridges and `/drone11`). 

### Useful Commands

**Check State:**
```bash
ros2 topic echo /mavros/state
```

**Takeoff (if not using the test script):**
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 2.0}"
```

**Send Position Command:**
```bash
ros2 topic pub --rate 20 /mavros/setpoint_position/local \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}"
```

**Land:**
```bash
ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{}"
```