#!/usr/bin/env python3
"""Demo flight: take off to 3 m, fly a 3 x 3 m square, come back and land.

    dronelab.sh square --drone 11            # real drone 11 (asks for confirmation)
    dronelab.sh square --drone 1 --yes       # the simulated drone of the Demo
    python3 -m dronelab.demos.square --help  # inside the container (DroneLab Shell)

It flies through the STUDENT interface (/droneNN/... on the students' ROS domain), exactly like a
student program would, so the safety gate checks every command - it doubles as an example for
students. Position setpoints move a "carrot" along each edge at --speed, so the speed is limited
even when ArduPilot's WPNAV_SPEED is high.

The square is placed where it fits inside the fence (with --clearance to every wall) and the
altitude is lowered below the fence ceiling if needed (the indoor ceiling is 2.5 m).

Taking over: switch the RC to LOITER (or any other mode) at any time - the demo notices that the
drone left GUIDED and stops sending anything. Ctrl+C (SIGINT/SIGTERM/SIGHUP) lands the drone.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from typing import List, Optional, Tuple

from ..config import Config
from ..gate.admin import AdminClient
from ..geo import Vec3
from .. import geo
from ..safety import BoxFence, CircleFence, Fence, PolygonFence

XY = Tuple[float, float]


class Abort(Exception):
    """Stop the demo. land=True: we still own the flight (GUIDED) -> land instead of leaving it hovering.
    land=False: someone else took over (RC pilot, instructor, failsafe) -> send nothing more."""

    def __init__(self, msg: str, land: bool = True):
        super().__init__(msg)
        self.land = land


# ----------------------------------------------------------------------------- geometry (pure)
def fence_from_status(d: dict) -> Fence:
    """Rebuild the gate's fence from its status["fence"] (Fence.describe())."""
    t = d.get("type")
    z = d.get("z", [0.0, 2.5])
    if t == "box":
        return BoxFence(d["x"][0], d["x"][1], d["y"][0], d["y"][1], z[0], z[1], d.get("margin", 0.3))
    if t == "polygon":
        return PolygonFence(points=[tuple(p) for p in d["points"]], zmin=z[0], zmax=z[1],
                            margin=d.get("margin", 2.0), is_global=d.get("frame") == "global")
    if t == "circle":
        return CircleFence(lat=d["center"][0], lon=d["center"][1], radius=d["radius"], zmin=z[0], zmax=z[1],
                           margin=d.get("margin", 2.0))
    raise ValueError(f"unknown fence {d}")


def clearance(fence: Fence, p: XY) -> float:
    """Distance from p to the nearest wall, negative outside (fence coordinates)."""
    if isinstance(fence, BoxFence):
        return min(p[0] - fence.xmin, fence.xmax - p[0], p[1] - fence.ymin, fence.ymax - p[1])
    if isinstance(fence, PolygonFence):
        d = geo.dist_to_polygon_edge(p, fence.poly)
        return d if geo.point_in_polygon(p, fence.poly) else -d
    if isinstance(fence, CircleFence):
        return fence.radius - math.hypot(*p)          # the circle's plane is centred on it
    raise ValueError("unknown fence")


def plan_square(fence: Fence, start: XY, side: float, min_clear: float) -> Optional[List[XY]]:
    """Corners (fence coordinates) of a side x side square near ``start``, closed (ends at the first
    corner), every corner at least ``min_clear`` from the walls. Prefers squares that start at the
    take-off point; picks the one with the most room around it."""
    sx, sy = start
    cands = []
    for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):          # take-off point = first corner
        cands.append([(sx, sy), (sx + dx * side, sy), (sx + dx * side, sy + dy * side), (sx, sy + dy * side)])
    h = side / 2.0                                                  # take-off point = centre
    cands.append([(sx - h, sy - h), (sx + h, sy - h), (sx + h, sy + h), (sx - h, sy + h)])
    best, best_c = None, -1e9
    for i, c in enumerate(cands):
        cl = min(clearance(fence, p) for p in c)
        if cl < min_clear:
            continue
        if any(fence.check_path(Vec3(a[0], a[1], 0), Vec3(b[0], b[1], 0)) for a, b in zip(c, c[1:] + c[:1])):
            continue
        score = cl + (0.5 if i < 4 else 0.0)                          # small bonus: start where it took off
        if score > best_c:
            best, best_c = c, score
    return None if best is None else best + [best[0]]


MIN_HEIGHT = 0.8        # never fly the square lower than this above the take-off point


def plan_height(fence: Fence, ground_fz: float, wanted: float, headroom: float = 0.3) -> float:
    """Height above the TAKE-OFF POINT to fly at. The fence checks ground_fz + height (fence frame), so it
    must stay within [zmin + 0.2, zmax - headroom]. Raises ValueError when there is no room - e.g. indoors
    with a drifted height reference (the drone on the floor reads z = 2 m under a 2.5 m ceiling)."""
    hmax = fence.zmax - headroom - ground_fz
    hmin = max(MIN_HEIGHT, fence.zmin + 0.2 - ground_fz)
    if hmax < hmin:
        raise ValueError(f"no room: on the ground the drone reads z = {ground_fz:+.2f} m, the fence allows "
                         f"z <= {fence.zmax:g} m, so it could climb only {max(hmax, 0):.1f} m (needs {hmin:.1f} m). "
                         f"If the drone really stands on the floor, its height reference is off "
                         f"(barometer drift? use OptiTrack height EK3_SRC1_POSZ=6, or reboot the flight controller)")
    return min(max(wanted, hmin), hmax)


def fit_altitude(fence: Fence, wanted: float, headroom: float = 0.3) -> float:
    """Fence-frame altitude for a drone taking off from z = 0."""
    return plan_height(fence, 0.0, wanted, headroom)


# ----------------------------------------------------------------------------- ROS side
class Pilot:
    """Minimal student client: everything goes through the gate."""

    def __init__(self, domain: int, ns: str, log):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
        from std_msgs.msg import String
        self.PoseStamped, self.CommandBool, self.CommandTOL, self.SetMode = PoseStamped, CommandBool, CommandTOL, SetMode
        self.log = log
        self.ns = ns
        self.ctx = Context()
        self.ctx.init(domain_id=domain)
        n = self.n = Node("square_demo", context=self.ctx)
        self.pose: Optional[Vec3] = None
        self.orient = None
        self.hold_orient = None
        self.state = None
        self.status: dict = {}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(PoseStamped, f"{ns}/local_position/pose", self._on_pose, qos_profile_sensor_data)
        n.create_subscription(State, f"{ns}/state", lambda m: setattr(self, "state", m), 10)
        n.create_subscription(String, f"{ns}/gate/status", self._on_status, latched)
        n.create_subscription(String, f"{ns}/error", self._on_error, 50)
        self.p_sp = n.create_publisher(PoseStamped, f"{ns}/setpoint_position/local", 10)
        self.c_arm = n.create_client(CommandBool, f"{ns}/cmd/arming")
        self.c_mode = n.create_client(SetMode, f"{ns}/set_mode")
        self.c_takeoff = n.create_client(CommandTOL, f"{ns}/cmd/takeoff")
        self.c_land = n.create_client(CommandTOL, f"{ns}/cmd/land")
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(n)
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while self._running:
            try:
                self.ex.spin_once(timeout_sec=0.1)
            except Exception:
                if not self._running:
                    break

    def _on_pose(self, m):
        p = m.pose.position
        self.pose = Vec3(p.x, p.y, p.z)
        self.orient = m.pose.orientation

    def _on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def _on_error(self, m):
        self.log("  gate: " + m.data)

    # ---- services
    def _call(self, cli, req, timeout=15.0):
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def set_mode(self, mode: str) -> bool:
        r = self.SetMode.Request()
        r.custom_mode = mode
        res = self._call(self.c_mode, r)
        return bool(res and res.mode_sent)

    def arm(self) -> bool:
        r = self.CommandBool.Request()
        r.value = True
        res = self._call(self.c_arm, r, timeout=25.0)      # the gate runs LOITER -> ARM -> GUIDED
        return bool(res and res.success)

    def takeoff(self, alt: float) -> bool:
        r = self.CommandTOL.Request()
        r.altitude = float(alt)
        res = self._call(self.c_takeoff, r)
        return bool(res and res.success)

    def land(self) -> bool:
        res = self._call(self.c_land, self.CommandTOL.Request())
        return bool(res and res.success)

    def goto(self, p: Vec3):
        m = self.PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p.x), float(p.y), float(p.z)
        if self.hold_orient is not None:
            m.pose.orientation = self.hold_orient     # keep the heading it had at the start
        else:
            m.pose.orientation.w = 1.0
        self.p_sp.publish(m)

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)
        try:
            self.ex.shutdown(timeout_sec=1.0)
            self.n.destroy_node()
            self.ctx.try_shutdown()
        except Exception:
            pass


class SquareFlight:
    def __init__(self, a, log=print):
        self.a = a
        self.log = log
        self.pilot: Optional[Pilot] = None
        self.flying = False          # we armed/took off and still own the flight
        self.min_z = -1e9            # lowest setpoint z allowed (student frame)
        self.stop = threading.Event()

    # ---- state checks (called in every loop iteration)
    def guard(self, need_air=True):
        if self.stop.is_set():
            raise Abort("stopped")
        st, s = self.pilot.state, self.pilot.status
        if st is None or not st.connected:
            raise Abort("lost the drone (no /state)", land=False)
        if s.get("locked"):
            raise Abort(f"the gate locked student commands: {s.get('lock_reason')}", land=False)
        if s.get("pilot_control"):
            raise Abort(f"the RC pilot / a failsafe took over ({s.get('pilot_control')}) - nothing more is sent",
                        land=False)
        if need_air:
            if not st.armed:
                raise Abort("the drone disarmed", land=False)
            if st.mode != "GUIDED":
                raise Abort(f"the drone left GUIDED (now {st.mode}): RC pilot, instructor or failsafe took over - "
                            f"nothing more is sent", land=False)

    def wait(self, cond, timeout, what, need_air=True, step=0.1, guard=True):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if guard:
                self.guard(need_air)
            elif self.stop.is_set():
                raise Abort("stopped")
            if cond():
                return
            time.sleep(step)
        raise Abort(f"timeout: {what}")

    def run(self) -> int:
        a = self.a
        cfg = Config.load()
        ds = cfg.drone(a.drone)
        domain, ns = a.domain, ds.student_ns
        adm = AdminClient(ds.socket_path)       # same machine as the gate: ask it where students live
        t0 = time.time()
        while time.time() - t0 < 2.0 and not adm.last_status:
            adm.poll()
            time.sleep(0.1)
        if adm.last_status:
            ns = adm.last_status.get("student_ns", ns)
            if domain is None:
                domain = int(adm.last_status.get("student_domain", ds.student_domain))
        adm.close()
        if domain is None:
            domain = ds.student_domain
        self.log(f"drone {a.drone}: {ns} on ROS domain {domain}")
        p = self.pilot = Pilot(domain, ns, self.log)
        try:
            self.wait(lambda: p.state is not None and p.state.connected and p.status and p.pose is not None, 30,
                      "no data from the gate - is the drone connected in the Flight app / Demo running?",
                      guard=False)
            self.wait(lambda: p.status.get("pos") is not None, 15,
                      "the gate has no position of the drone (OptiTrack / GPS / EKF not ready?)", guard=False)
            return self._fly()
        except Abort as e:
            self.log(f"STOPPED: {e}")
            st = p.state
            if e.land and self.flying and st is not None and st.armed and st.mode == "GUIDED":
                self.log("the demo still owns the flight -> LANDING")
                if not p.land():
                    self.log("LAND was rejected - take over with the RC!")
            return 2
        finally:
            p.close()

    def _fly(self) -> int:
        a, p = self.a, self.pilot
        s = p.status
        fence = fence_from_status(s["fence"])
        if s.get("ground_problem") and not p.state.armed:
            raise Abort(s["ground_problem"], land=False)
        lim = float((s.get("limits") or {}).get("max_speed_xy", a.speed))
        speed = min(a.speed, lim)
        fpos = Vec3(*s["pos"])
        lp = p.pose
        corners = plan_square(fence, (fpos.x, fpos.y), a.side, a.clearance)
        if corners is None:
            raise Abort(f"a {a.side:g} m square with {a.clearance:g} m clearance does not fit inside the fence "
                        f"around the drone ({fpos.x:.1f}, {fpos.y:.1f}) - move the drone or use --side", land=False)
        st = p.state
        # Heights. On the ground: everything relative to the take-off point (that is what cmd/takeoff uses),
        # checked against the fence in its own frame. Already flying: the fence frame decides.
        if not st.armed:
            try:
                h = plan_height(fence, fpos.z, a.alt)
            except ValueError as e:
                raise Abort(str(e), land=False)
            z_sq = lp.z + h                          # student frame
            self.min_z = lp.z + 0.5                  # never send a setpoint lower than this
            what = f"{h:.1f} m above the take-off point"
        else:
            alt = fit_altitude(fence, a.alt)
            z_sq = lp.z + (alt - fpos.z)
            self.min_z = lp.z + (fence.zmin + 0.2 - fpos.z)
            h = None
            what = f"z = {alt:.1f} m (fence frame)"
        if a.alt > (h if h is not None else alt) + 1e-6:
            self.log(f"NOTE: {a.alt:g} m does not fit under the fence ceiling ({fence.zmax:g} m): flying at {what}")
        # student (local) xy = fence xy - offset (0 indoors; GPS vs EKF origin outdoors)
        off = (fpos.x - lp.x, fpos.y - lp.y)
        pts = [Vec3(x - off[0], y - off[1], z_sq) for x, y in corners]
        home = Vec3(lp.x, lp.y, z_sq)                 # above the take-off point: it lands where it started
        self.log(f"square {a.side:g} x {a.side:g} m at {what}, {speed:.1f} m/s, corners (fence frame): "
                 + " ".join(f"({x:.1f},{y:.1f})" for x, y in corners[:-1]))
        if not a.yes and sys.stdin.isatty():
            ans = input(f"Drone {a.drone} ({st.mode}, {'ARMED' if st.armed else 'disarmed'}) will "
                        f"{'take off and ' if not st.armed else ''}fly this square. Area clear? [y/N] ")
            if ans.strip().lower() not in ("y", "yes"):
                self.log("cancelled")
                return 1
        p.hold_orient = p.orient

        # ---- 1) arm + take off (or start from the current hover)
        if not st.armed:
            self.log("arming ...")
            t0 = time.time()
            while True:
                self.guard(need_air=False)
                if p.arm():                       # real drones: the gate runs LOITER -> ARM -> GUIDED
                    break
                if time.time() - t0 > a.arm_timeout:
                    raise Abort("could not arm (pre-arm checks / EKF not ready?) - see the gate's errors above")
                self.log("  not armed yet, retrying (pre-arm checks / EKF) ...")
                time.sleep(3.0)
            self.flying = True
            t0 = time.time()
            while p.state.mode != "GUIDED":       # simulator: armed in its current mode
                self.guard(need_air=False)
                if time.time() - t0 > 8.0:
                    raise Abort(f"armed, but GUIDED was not accepted (mode {p.state.mode})")
                p.set_mode("GUIDED")
                time.sleep(0.5)
            self.log(f"taking off to {h:.1f} m ...")
            if not p.takeoff(h):
                raise Abort("take-off rejected (see the gate's errors above)")
            try:
                self.wait(lambda: p.pose.z >= z_sq - 0.3, 40, "climb")
            except Abort as e:
                if str(e).startswith("timeout"):
                    raise Abort(f"climb timeout: reached {p.pose.z - lp.z:.1f} of {h:.1f} m above the take-off point")
                raise
        else:
            st = p.state
            if st.mode != "GUIDED":
                raise Abort(f"the drone is armed in {st.mode}: switch it to GUIDED first")
            self.flying = True
        # ---- 2) the square
        here = p.pose
        self.fly_line(here, Vec3(pts[0].x, pts[0].y, pts[0].z), speed, "start corner")
        for i, (a_, b_) in enumerate(zip(pts, pts[1:])):
            self.fly_line(a_, b_, speed, f"corner {i + 2}/4" if i < 3 else "back to corner 1")
        if math.dist((pts[-1].x, pts[-1].y), (home.x, home.y)) > 0.3:
            self.fly_line(pts[-1], home, speed, "back above the take-off point")
        # ---- 3) land (or hover)
        if a.no_land:
            self.log("done - hovering in GUIDED (--no-land)")
            return 0
        self.log("landing ...")
        if not p.land():
            raise Abort("land rejected")
        self.wait(lambda: not p.state.armed, 90, "landing", need_air=False)
        self.flying = False
        self.log("landed and disarmed - done")
        return 0

    def fly_line(self, a: Vec3, b: Vec3, speed: float, label: str):
        p = self.pilot
        length = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        dur = max(length / max(speed, 0.05), 0.1)
        self.log(f"-> {label} ({b.x:.1f}, {b.y:.1f}, {b.z:.1f})")
        t0 = time.time()
        if b.z < self.min_z:
            raise Abort(f"refusing to fly to z = {b.z:.2f} (lower than {self.min_z:.2f}, 0.5 m above the take-off "
                        f"point) - height reference changed?")
        while True:
            self.guard()
            k = min(1.0, (time.time() - t0) / dur)
            p.goto(Vec3(a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k, max(a.z + (b.z - a.z) * k, self.min_z)))
            if k >= 1.0 and math.dist((p.pose.x, p.pose.y, p.pose.z), (b.x, b.y, b.z)) < self.a.tolerance:
                break
            if time.time() - t0 > dur + 20:
                raise Abort(f"did not reach {label} (blocked by the gate? see errors above)")
            time.sleep(0.1)
        t1 = time.time()
        while time.time() - t1 < self.a.hold:
            self.guard()
            p.goto(b)
            time.sleep(0.1)

    def emergency_land(self):
        """Ctrl+C / window closed: land if we still own the flight (GUIDED)."""
        p = self.pilot
        if p and self.flying and p.state is not None and p.state.armed and p.state.mode == "GUIDED":
            self.log("interrupted - LANDING")
            p.land()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Take off, fly a square through the safety gate, land.")
    ap.add_argument("--drone", type=int, default=1, help="MAVLink system id (1 = simulator, default)")
    ap.add_argument("--alt", type=float, default=3.0, help="altitude above take-off, m (lowered to fit the fence)")
    ap.add_argument("--side", type=float, default=3.0, help="side of the square, m")
    ap.add_argument("--speed", type=float, default=1.0, help="m/s (capped by the gate's limit)")
    ap.add_argument("--clearance", type=float, default=0.5, help="minimum distance of every corner to the fence, m")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hover at each corner")
    ap.add_argument("--tolerance", type=float, default=0.35, help="corner reached within this distance, m")
    ap.add_argument("--arm-timeout", type=float, default=90.0)
    ap.add_argument("--domain", type=int, default=None, help="students' ROS domain (default: ask the gate)")
    ap.add_argument("--no-land", action="store_true", help="hover at the end instead of landing")
    ap.add_argument("--yes", "-y", action="store_true", help="do not ask for confirmation")
    a = ap.parse_args(argv)
    os.environ.pop("CYCLONEDDS_URI", None)       # the students' (network) DDS setup

    def log(msg):
        print(time.strftime("%H:%M:%S ") + msg, flush=True)

    f = SquareFlight(a, log)

    def on_signal(signum, _frame):
        f.stop.set()
        f.emergency_land()
        os._exit(130)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)
    return f.run()


if __name__ == "__main__":
    sys.exit(main())
