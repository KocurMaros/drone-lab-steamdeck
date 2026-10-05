# DroneLab on the Steam Deck

Two touch-friendly apps for the Steam Deck (SteamOS desktop mode). Everything runs in one
container that keeps running in the background.

| Icon | What it does |
|------|--------------|
| **DroneLab Flight** | Finds real drones on whatever subnet/port they use, starts MAVROS for them and puts a **safety gate** between the students and the drone (geofence, speed limits, error topic, breach → LAND). |
| **DroneLab Demo** | One-tap simulation for visitors: Gazebo arena with a gate race, gamepad flying (Steam Deck controls or any pad), live FPV camera with HUD, minimap, leaderboard. Gazebo's chase view goes to a second screen. |

Lab hardware procedures (OptiTrack/Motive, drone network, transmitter pairing, emergency
procedures) are in [docs/lab-setup.md](docs/lab-setup.md).

---

## Install (once, on the Deck)

```bash
git clone https://github.com/KocurMaros/drone-lab-steamdeck.git ~/Desktop/dronelab   # any folder works
~/Desktop/dronelab/scripts/install-desktop.sh
```

This puts **DroneLab Flight**, **DroneLab Demo** and **DroneLab Shell** in the menu and on the Desktop.
The first start builds the container image (`podman`, preinstalled on SteamOS 3.5+; 30–60 min the
first time, a Konsole window shows progress). Later starts take seconds. After `git pull` no
rebuild is needed unless the `Dockerfile` changed (the launcher notices and rebuilds).

For the Deck's own sticks/buttons in the Demo, add `scripts/dronelab-demo-steam.sh` to Steam as a
*Non-Steam Game* and start it from Steam (controller layout "Gamepad"). External USB/Bluetooth pads
also work from the desktop icon.

Terminal equivalents: `scripts/dronelab.sh flight | demo | shell | build | stop | status`.

---

## DroneLab Flight (real drones)

```
 students (lab Wi-Fi, ROS_DOMAIN_ID 0)          Steam Deck                          drone
 ───────────────────────────────────           ───────────────────────────          ─────
 /drone11/setpoint_position/local  ──►  gate: fence + limits + rules  ──►  MAVROS  ──►  ArduPilot
 /drone11/cmd/arming, set_mode ...  ──►  (rejects → /drone11/error)       private ROS domain 81,
 /drone11/<every MAVROS topic>     ◄──  relay                        ◄──  loopback only
```

1. Power the drone. It appears under **FOUND DRONES** with its system ID, mode and how it was reached.
2. Pick the fence for new connections (top bar; *drone default* = from the config).
3. **CONNECT**. The gate starts MAVROS on a private ROS domain that is bound to the Deck's loopback
   interface (students cannot reach MAVROS at all) and exposes the student interface below.
4. The drone tab shows mode/arming/battery/link/position age/fence state, a top-down fence map with the
   drone, which student topics are active, and every rejected or limited command.
5. **LAND** / **BRAKE** / **LOITER** act immediately; **KILL** (hold 2 s) cuts the motors;
   **LAND ALL** (top right) lands every connected drone. RTL/disarm/log/disconnect are under `···`.

Closing the app asks whether to keep the gates running. Keeping them keeps the fence active; reopening
the app re-attaches.

### How drones are found (different subnet/port every time)

* **listen** – the Deck listens on UDP `14510-14519` and `14540-14559`. Any drone whose mavlink-router
  pushes to the Deck (`Mode = Normal` endpoint to the Deck's IP) shows up, on any port in those ranges.
* **probe** – every 3 s the Deck sends a GCS heartbeat to `<drone>:14550` on `192.168.18.110-119`,
  `10.42.0.1`, `192.168.55.1` **and every /24 the Deck is currently on**. Drones with a
  `Mode = Server` endpoint answer, so a new subnet needs no config change.
* Still nothing? **Manual connection…** takes any MAVROS `fcu_url` (UDP, TCP, serial).

Ports, targets and timings are in `config/dronelab.yaml` → `network`. Every drone needs a unique
`SYSID_THISMAV` (= drone ID); two drones with the same ID are shown as one.

### Student interface (ROS_DOMAIN_ID 0)

Same names and types as MAVROS, with `/drone<ID>` instead of `/mavros`. Code written against the
simulation works unchanged on the real drone (the Demo's drone is `/drone1`).

| Topic / service | Type | Gate behaviour |
|---|---|---|
| `/droneNN/<every MAVROS topic>` (state, local_position/pose, battery, imu/data, statustext/recv, …) | as MAVROS | relayed (local poses in the arena frame) |
| `setpoint_position/local` | `geometry_msgs/PoseStamped` | target must be inside the fence (and above `z_min`), else dropped + error. All-zero quaternion = keep heading |
| `setpoint_velocity/cmd_vel_unstamped`, `setpoint_velocity/cmd_vel` | `Twist` / `TwistStamped` (ENU) | speed limited and slowed to a stop `stop_buffer_m` inside the fence. A rejected velocity becomes a hover, never a silent drop (ArduPilot would keep flying the old one for 3 s). **Stream at ≥ 5 Hz**: 0.5 s after the last velocity the gate commands a hover |
| `setpoint_raw/local` | `mavros_msgs/PositionTarget` | frames 1, 7, 8, 9; full position or full velocity; acceleration/force rejected (with a hover); a feed-forward velocity next to a position is removed (ArduPilot would push the target through the fence) |
| `setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | outdoor fences only. **Altitude is relative to home**, not AMSL |
| `setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | outdoor only, `coordinate_frame` 6, position only |
| `cmd/arming` | `mavros_msgs/CommandBool` | `true`: LOITER → ARM → GUIDED (real drones cannot arm in GUIDED). `false` in the air = LAND instead of cutting the motors |
| `set_mode` | `mavros_msgs/SetMode` | allow-list per fence profile (indoor: GUIDED, LOITER, LAND, BRAKE; RTL only outdoors/sim); GUIDED only once armed. While armed, students cannot leave LAND/RTL/BRAKE that the instructor, the RC or a failsafe selected |
| `cmd/takeoff` | `mavros_msgs/CommandTOL` | armed + GUIDED + altitude within the fence |
| `cmd/land` | `mavros_msgs/CommandTOL` | always allowed |
| `error` | `std_msgs/String` | every rejection / limitation, rate limited per error type |
| `gate/status` | `std_msgs/String` (JSON) | fence, limits, lock state, counters |
| `gate/fence` | `visualization_msgs/Marker` | fence outline for RViz (`map` frame) |

Not exposed on purpose: `cmd/command` (arbitrary MAVLink commands), parameter services, RC override.
After `cmd/arming` true, take off within ~10 s or ArduPilot disarms again.

### Fence and breach

Fence profiles live in `config/dronelab.yaml` → `fence_profiles`: `box` (indoor, metres in the arena
frame), `polygon` (lat/lon corners, or local metres) or `circle`, each with a `margin_m`. Profiles marked
`verified: false` (the outdoor example!) show a warning and need a confirmation before connecting.
`arena_to_local` maps the arena/OptiTrack frame to the drone's EKF frame if they differ.

The gate also watches the **measured** position (it asks ArduPilot for 20 Hz position and for the landed
state). If the drone is outside fence + margin for 3 samples while airborne, it switches to **LAND** (re-sent
until the drone reports LAND) and locks all student commands except landing until you press
**RELEASE LOCK**. After the release it stays quiet until the drone has been back inside the fence, so it
can be flown back in. It does not fight your transmitter: once LAND was reached, a mode you select with
the RC is left alone. The instructor buttons **LAND / BRAKE / LOITER / RTL** also lock student commands.

Tuning near walls: allowed speed `v` satisfies `v·lookahead_s + v²/(2·decel) ≤ distance − stop_buffer_m`.
In SITL a 6 m/s approach stopped 0.3 m inside the wall with the `sim_arena` values. **Check the indoor
values on the real drone** (fly slowly at a wall with the RC ready) before students use it.

### What the gate does NOT protect against (read this)

* **MAVLink sent straight to the drone.** The drone's mavlink-router still accepts MAVLink from anyone on
  the network on its `Server` endpoint (`gcs_radio`, `0.0.0.0:14550` in `drones/main.conf`). A student
  who sends to `<drone>:14550` bypasses the Deck completely, and also takes the Server endpoint's
  only client slot away from the Deck. For student sessions, remove Server endpoints from the drone and
  use a `Normal` endpoint to the Deck's fixed IP (DHCP reservation on the lab router).
* **Deck or Wi-Fi failure.** If the link drops, the gate can do nothing. Configure ArduPilot's own
  failsafes and fence; the attached `drones/edu10/*.params` have `FENCE_ENABLE 0` and
  `FS_GCS_ENABLE 0`. MAVROS identifies as system 255 (= `SYSID_MYGCS`), so with `FS_GCS_ENABLE` set the
  drone lands/RTLs when the Deck stops sending heartbeats.
* **Determined attackers on the network.** The private domain blocks normal ROS 2 discovery from other
  machines; it is not a security boundary against someone crafting DDS packets.
* **Outdoor flights.** The example outdoor polygon is not your field. EU open-category rules apply
  (e.g. 120 m max height, VLOS, registered operator); a software fence does not replace them.

The transmitter (kill switch, mode switch) always stays the primary safety device.

---

## DroneLab Demo (simulation for visitors)

Tap **DroneLab Demo**. It starts Gazebo, ArduCopter SITL and the same safety gate (fence =
the arena), then shows the pilot screen on the Deck and Gazebo's chase camera on the second screen
(if one is connected).

| Gamepad | Keyboard | Action |
|---|---|---|
| A / Start | Space | take off (and start the race timer) |
| B | L | land |
| X | R | reset: drone back on the pad (also automatic after a crash) |
| Y | Tab | race ↔ free flight |
| View/Back | Esc | beginner ↔ expert speed |
| left stick | ↑ ↓ ← → | climb/descend, turn |
| right stick | W A S D | forward/back, left/right |
| right trigger | Shift | boost (expert) |

**Race:** fly through the 8 coloured gates in order (HUD brackets the next gate, minimap shows the
course), then land on the H pad; the time stops at touchdown and goes to the leaderboard
(`var/leaderboard.json`, rename/clear via ☰). The invisible fence is the real gate code slowing the
drone down; visitors can't fly out of the arena.

Students can use the running simulation exactly like a real drone: `/drone1/...` on ROS_DOMAIN_ID 0,
plus `/drone1/camera/image_raw`.

The arena is generated from `sim/course/arena.yaml` (gates, buildings, trees, fence) by
`python3 tools/gen_arena.py`; the minimap and race logic read the same file.

---

## Testing

* `python3 -m pytest tests` – fence geometry, command validation, discovery (against pymavlink frames),
  race logic, config consistency (49 tests, no ROS needed).
* `scripts/dronelab.sh selftest` – headless end-to-end run in the container: Gazebo + ArduCopter SITL +
  MAVROS behind the gate, flown through the student API. Checks isolation, arming/takeoff rules, wall
  slow-down, the 0.5 s hover watchdog, rejected setpoints, feed-forward removal, mode rules, breach → LAND
  + lock, release, landing. Takes ~4 min; stop the Demo first.

## Files

```
scripts/dronelab.sh            host launcher (podman/docker), used by the desktop icons
scripts/in-container.sh        environment inside the container
config/dronelab.yaml           network discovery, fence profiles, limits, demo settings
dronelab/                      Python package (no colcon build needed)
  gate/                        safety gate (one process per drone, starts MAVROS)
  safety.py, geo.py            fence + command validation (pure Python, unit tested)
  discovery.py, mavlink_lite.py   drone discovery
  gui/flight_app.py, gui/demo_app.py
  sim/stack.py, sim/race.py    simulation processes, race logic
sim/                           Gazebo world, drone model with FPV camera, SITL params, course
tests/                         pytest (python3 -m pytest tests)
legacy/                        the previous per-process launchers (unused)
var/                           logs, leaderboard (not in git)
```

Logs: `var/logs/` (`gate-<ID>.log`, `mavros-<ID>.log`, `gzserver.log`, `sitl.log`, app logs), also
from the apps' menus.

## Troubleshooting

* **No drone found** – check the Deck's IP (top bar) is in the drone's subnet; `ip a` in DroneLab Shell;
  on the drone `systemctl status mavlink-router`. Use Manual connection with the drone's IP.
* **"ports already in use"** – another MAVROS/QGroundControl holds them (`scripts/dronelab.sh stop`
  stops everything in the container).
* **Gate exits right away** – Flight app shows the log tail; usually MAVROS missing in the image or a
  config error (`config/dronelab.yaml` is validated at start).
* **Demo is slow** – Gazebo needs the GPU: the launcher passes `/dev/dri`; check `glxinfo -B` in DroneLab
  Shell shows AMD, not llvmpipe. Close the chase view (☰) if needed.
* **No gamepad in the Demo** – start it from Steam (see Install) or plug in a pad; the status line in the
  bottom right says what was detected.
* **Reset takes ~10-20 s** – that is ArduPilot's EKF accepting the jump back to the pad.
* **Gazebo chase view covers the pilot screen** – it only opens automatically with a second screen
  (`demo.show_gazebo: auto`); ☰ → *Show Gazebo chase view* forces it.
