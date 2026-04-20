# DroneLab Setup & Execution Guide

This guide details the complete step-by-step process for setting up the DroneLab environment, from configuring the OptiTrack system on the Desktop PC to executing your first ROS-controlled flight.

## 🌐 System Architecture & Network

**Communication Flow:**
`Motive PC (Desktop) <-> DRONE <-> Connector (SteamDeck) <-> Your PC`

**Network Requirements:**
- All devices must be connected to the same network.
- Recommended/Tested Networks: `OSK_24` or `OSK_5`.
- IP Range: `192.168.18.1/24`

---

## Phase 1: OptiTrack Desktop PC Setup

### 1. Network Configuration
Set a static IP on the Desktop PC running Motive:
- **IP Address:** `192.168.18.200`
- **Subnet Mask:** `255.255.255.0`

### 2. Motive Streaming Settings
Open Motive and navigate to the **Settings -> Streaming** tab. Configure the following:
- **Enable:** Checked
- **Local Interface:** `192.168.18.200`
- **Transmission Type:** Multicast
- **Up Axis:** Y-Axis
- **Advanced Settings:**
  - **Data Port:** `1511`
  - **Command Port:** `1510`
  - **Multicast Interface:** `239.255.42.99`

### 3. Drone Marker Placement & Rigid Body Creation
To track the drone, you must attach reflective markers.

**Requirements:**
- Minimum 3 markers for basic tracking, minimum 4 for rigid body creation.
- Recommended: 10+ markers for robust, occlusion-resistant tracking.

**Placement Recommendations:**
- Place markers asymmetrically (avoids orientation flipping).
- Mount them at different heights (e.g., on top of the battery, on the arms, and underneath) to create a 3D volume.
- Ensure they are rigidly attached so they do not vibrate during flight.

**Creating the Rigid Body:**
1. Place the drone in the center of the tracking space (0, 0, 0).
2. Ensure the drone is on the ground, facing the Z-arrow.
3. In Motive, box-select all the markers on the drone.
4. Right-click and select **Create Rigid Body**.
5. In the Assets Menu, change the **Name** and **Streaming ID** to match the Drone ID (e.g., 10 through 19). 
   *Note: Streaming IDs must be unique for multiple devices flying simultaneously.*

### 4. Aligning the Axes
Open the Rigid Body Builder and align the orientation axes:
- **X Axis (Red Arrow):** Match to OptiTrack Left.
- **Y Axis (Green Arrow):** Match to OptiTrack UP.
- **Z Axis (Blue Arrow):** Match to OptiTrack Drone Front.

---

## Phase 2: MAVProxy Integration

Once Motive is streaming, you need to connect it to MAVProxy.

Run the MAVProxy script:
```bash
mavproxy.py
```
*Note: It will automatically discover the connected device on port `14550`. If running multiple devices, you must start MAVProxy via the terminal with specific port input parameters.*

In the MAVProxy console, execute the following commands to initialize OptiTrack:
```bash
module load optitrack
optitrack set obj_id 10  # Replace '10' with your actual Streaming ID/Drone ID
optitrack set client 192.168.18.200
optitrack set server 192.168.18.200
optitrack start
```
**Verification:** Check the white MAVProxy console. You should see `pre-arm good` and a continuous feed of estimated x, y, z positions.

---

## Phase 3: Drone-Side Configuration

### 1. Drone IP Setup
Connect the drone to the network and set a static IP in the format `192.168.18.1xx` (where `xx` is your Drone ID).

### 2. MAVLink Router Configuration
You must configure endpoints so the drone routes data to the connector.

Edit the configuration file:
```bash
sudo nano /etc/mavlink-router/main.conf
```
Ensure endpoints are created targeting the drone-connector (`192.168.18.100`) on port `145xx` (where `xx` is the Drone ID) for the `OSK_24` or `OSK_5` interfaces.

After any config changes, restart and verify the service:
```bash
sudo systemctl restart mavlink-router.service
sudo systemctl status mavlink-router.service
```
*(If this service is not running, no telemetry data will transfer to the PC).*

---

## Phase 4: ROS2 Connector

The Connector bridges MAVLink messages to ROS2 topics and enforces flight boundary checks.

Run the connector node:
```bash
ros2 run drone_lab_connector drone_lab_connector
```
Follow the GUI/terminal prompts to select the specific drone you want to communicate with. You can now subscribe to and publish ROS2 messages on your personal PC.

---

## Phase 5: Transmitter Binding & Pre-Flight

Each drone is paired with a specific transmitter. Reference the map below:

| Drone ID | Transmitter |
|----------|-------------|
| Drone 10 | TX1         |
| Drone 11 | TX3         |
| Drone 13 | TX2         |
| Drone 15 | TX1         |
| Drone 17 | TX2         |
| Drone 18 | TX3         |
| Drone 19 | TX3         |

### Receiver LED Status Guide
While the drone is booting, you must bind it to the transmitter. Look at the small green LED on the back of the drone:
- **Slow Blinking:** Ready to bind.
- **Fast Blinking:** Creating configuration hotspot.
- **Static Solid:** Successfully bound to transmitter.

---

## Phase 6: Flight Operations

Once all setup steps are complete and the LED is solid green, the drone is ready to fly.

### Flight Controls & Arming

**IMPORTANT:** On real drones you **cannot** arm in GUIDED mode. You must arm in **LOITER** mode first.

The connector enforces this sequence automatically – it will reject arming in GUIDED and reject switching to GUIDED before arming.

#### Manual Arming (Transmitter)
1. Switch **SE** to **LOITER** mode.
2. Toggle **SF Switch** to arm. Keep it armed.
3. Move throttle to mid-stick (0% mixer) for takeoff.
4. Once airborne and stable, switch **SE** to **GUIDED** for ROS2 position control.

#### ROS2 Arming & Takeoff (Command Line)

Replace `eduXX` with your drone ID (e.g. `edu11`).

**Step 1 – Set LOITER mode:**
```bash
ros2 service call /drones/eduXX/safe/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LOITER'}"
```

**Step 2 – Arm the drone:**
```bash
ros2 service call /drones/eduXX/safe/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

**Step 3 – Takeoff (altitude in metres):**
```bash
ros2 service call /drones/eduXX/safe/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 1.0}"
```

**Step 4 – Switch to GUIDED for position control:**
```bash
ros2 service call /drones/eduXX/safe/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

---

## Phase 7: Sending Position Commands via ROS2

### Architecture

All student communication goes through the **safety/relay node**. Students never interact with MAVROS directly. The connector validates setpoints, enforces flight boundaries, and proxies all services.

```
Student PC  ──►  /drones/eduXX/...  ──►  Safety Node  ──►  MAVROS (internal)  ──►  Drone
                 (student topics)         (validates)       (hidden topics)
```

### Student Topics (read directly from MAVROS)

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/drones/eduXX/setpoint_position/local` | `PoseStamped` | **Publish** | Position commands (safety-checked) |
| `/drones/eduXX/local_position/pose` | `PoseStamped` | Subscribe | Current drone position |
| `/drones/eduXX/state` | `mavros_msgs/State` | Subscribe | Drone state (armed, mode) |
| `/drones/eduXX/battery` | `sensor_msgs/BatteryState` | Subscribe | Battery voltage/percentage |
| `/drones/eduXX/error` | `std_msgs/String` | Subscribe | Safety errors & violations |
| `/drone_markers` | `MarkerArray` | Subscribe (RViz) | All drone positions |

### Student Services (safety-validated, use `safe/` prefix)

| Service | Type | Purpose |
|---------|------|---------|
| `/drones/eduXX/safe/cmd/arming` | `CommandBool` | Arm/disarm (blocked unless LOITER) |
| `/drones/eduXX/safe/set_mode` | `SetMode` | Change flight mode (GUIDED blocked unless armed) |
| `/drones/eduXX/safe/cmd/takeoff` | `CommandTOL` | Takeoff (altitude checked against safety bounds) |
| `/drones/eduXX/safe/cmd/land` | `CommandTOL` | Land (always allowed) |

> **Note:** MAVROS raw services (`/drones/eduXX/cmd/arming`, `/set_mode`, etc.) are also accessible but have **no safety validation**. Always use the `safe/` prefix for validated operations.

### Example: Full Flight Sequence

```bash
# 1. Monitor state
ros2 topic echo /drones/edu11/state

# 2. Set LOITER mode
ros2 service call /drones/edu11/safe/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LOITER'}"

# 3. Arm
ros2 service call /drones/edu11/safe/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# 4. Takeoff to 1m
ros2 service call /drones/edu11/safe/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 1.0}"

# 5. Switch to GUIDED
ros2 service call /drones/edu11/safe/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# 6. Send position setpoint (continuous stream required for GUIDED)
ros2 topic pub --rate 20 /drones/edu11/setpoint_position/local \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"

# 7. Land
ros2 service call /drones/edu11/safe/cmd/land mavros_msgs/srv/CommandTOL "{}"

# 8. Disarm (after landed)
ros2 service call /drones/edu11/safe/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

### Monitoring Commands

```bash
# Current position
ros2 topic echo /drones/edu11/local_position/pose

# Battery
ros2 topic echo /drones/edu11/battery

# Safety errors
ros2 topic echo /drones/edu11/error
```

### RViz Visualization
Add a **MarkerArray** display in RViz subscribed to `/drone_markers` to see all active drone positions with labels.

### Connector Configuration

The connector device IP is configurable in `default_config.yaml` or `~/.config/drone_lab_connector/user_param.yaml`:

```yaml
connector:
  ip: "192.168.18.201"
```

---

## 🚨 Emergency Procedures

If you lose control or the drone behaves erratically, immediately execute one of the following:

1. **Kill Switch:** Toggle the **SF Switch** to disarm the motors immediately.
2. **Manual Override:** Switch the **SE Switch** back to `ALT_HOLD` and take manual control of the drone using the transmitter sticks.

*You are now ready to publish movement commands via ROS on your PC!*
