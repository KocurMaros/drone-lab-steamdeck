#!/usr/bin/env python3
"""End-to-end self test of the simulation + safety gate (run inside the container: DroneLab Shell).

    python3 tools/e2e_sim_test.py            # starts Gazebo (headless) + SITL + gate, runs checks, stops
    python3 tools/e2e_sim_test.py --attach   # use a simulation that is already running (e.g. the Demo app)

Flies the simulated drone through the STUDENT interface (/drone1/... on the student domain) and checks
that the gate limits, rejects, hovers, and lands + locks on a fence breach; flies the 3 x 3 m square
demo; simulates an RC pilot taking over (LOITER + sticks via RC override on the private domain) and
checks that the gate never fights the pilot; reviews and fixes the ArduPilot parameters.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from dronelab.config import Config  # noqa: E402
from dronelab.dds import ros_env, write_cyclone_config  # noqa: E402
from dronelab.gate.admin import AdminClient  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)
    return ok


class Student:
    def __init__(self, domain):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data as SQ
        from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
        from mavros_msgs.msg import ExtendedState, PositionTarget, State
        from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        self.m = dict(PoseStamped=PoseStamped, Twist=Twist, PositionTarget=PositionTarget, CommandBool=CommandBool,
                      CommandTOL=CommandTOL, SetMode=SetMode)
        self.ctx = Context()
        self.ctx.init(domain_id=domain)
        n = self.n = Node("e2e_student", context=self.ctx)
        self.pose = None
        self.yaw = 0.0
        self.vel = (0, 0, 0)
        self.state = None
        self.landed = 0
        self.errors = []
        self.images = 0
        n.create_subscription(PoseStamped, "/drone1/local_position/pose", self._pose, SQ)
        n.create_subscription(TwistStamped, "/drone1/local_position/velocity_local",
                              lambda m: setattr(self, "vel", (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)), SQ)
        n.create_subscription(State, "/drone1/state", lambda m: setattr(self, "state", m), SQ)
        n.create_subscription(ExtendedState, "/drone1/extended_state",
                              lambda m: setattr(self, "landed", m.landed_state), SQ)
        n.create_subscription(String, "/drone1/error", lambda m: self.errors.append((time.monotonic(), m.data)), 50)
        n.create_subscription(Image, "/drone1/camera/image_raw", lambda m: setattr(self, "images", self.images + 1), SQ)
        self.p_pose = n.create_publisher(PoseStamped, "/drone1/setpoint_position/local", 10)
        self.p_vel = n.create_publisher(Twist, "/drone1/setpoint_velocity/cmd_vel_unstamped", 10)
        self.p_raw = n.create_publisher(PositionTarget, "/drone1/setpoint_raw/local", 10)
        self.c_arm = n.create_client(CommandBool, "/drone1/cmd/arming")
        self.c_mode = n.create_client(SetMode, "/drone1/set_mode")
        self.c_to = n.create_client(CommandTOL, "/drone1/cmd/takeoff")
        self.c_land = n.create_client(CommandTOL, "/drone1/cmd/land")
        self.ex = MultiThreadedExecutor(context=self.ctx)
        self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        self.rclpy = rclpy

    def _pose(self, m):
        self.pose = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        q = m.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def call(self, cli, req, timeout=15):
        if not cli.wait_for_service(timeout_sec=5):
            return None
        f = cli.call_async(req)
        t0 = time.time()
        while not f.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return f.result() if f.done() else None

    def mode(self, m):
        r = self.m["SetMode"].Request()
        r.custom_mode = m
        res = self.call(self.c_mode, r)
        return bool(res and res.mode_sent)

    def arm(self, v=True):
        r = self.m["CommandBool"].Request()
        r.value = v
        res = self.call(self.c_arm, r)
        return bool(res and res.success)

    def takeoff(self, alt):
        r = self.m["CommandTOL"].Request()
        r.altitude = float(alt)
        res = self.call(self.c_to, r)
        return bool(res and res.success)

    def land(self):
        res = self.call(self.c_land, self.m["CommandTOL"].Request())
        return bool(res and res.success)

    def vel_cmd(self, vx, vy, vz=0.0, yr=0.0):
        t = self.m["Twist"]()
        t.linear.x, t.linear.y, t.linear.z, t.angular.z = float(vx), float(vy), float(vz), float(yr)
        self.p_vel.publish(t)

    def goto(self, x, y, z):
        p = self.m["PoseStamped"]()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
        p.pose.orientation.w = 1.0
        self.p_pose.publish(p)

    def errors_since(self, t0, text):
        return [e for t, e in self.errors if t >= t0 and text in e]

    def wait(self, cond, timeout, step=0.1):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if cond():
                return True
            time.sleep(step)
        return False


class Private:
    """Talks to MAVROS directly on the private domain - plays the RC pilot / a bypassing program."""

    def __init__(self, domain):
        xml = write_cyclone_config("e2e-private", [domain], None)
        os.environ.update(ros_env(domain, xml))
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import OverrideRCIn
        from mavros_msgs.srv import SetMode
        self.PoseStamped, self.OverrideRCIn, self.SetMode = PoseStamped, OverrideRCIn, SetMode
        self.ctx = Context()
        self.ctx.init(domain_id=domain)
        n = self.n = Node("e2e_bypass", context=self.ctx)
        self.p_sp = n.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.p_rc = n.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.c_mode = n.create_client(SetMode, "/mavros/set_mode")
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        self.sticks = None                      # (roll, pitch, throttle, yaw) PWM, streamed at 10 Hz
        threading.Thread(target=self._rc_loop, daemon=True).start()

    def _rc_loop(self):
        while True:
            st = self.sticks
            if st is not None:
                m = self.OverrideRCIn()
                ch = [65535] * len(m.channels)          # CHAN_NOCHANGE
                ch[:4] = [int(v) for v in st]           # 0 = CHAN_RELEASE
                m.channels = ch
                self.p_rc.publish(m)
            time.sleep(0.1)

    def stick_toward(self, yaw, east, north, amount=350):
        """Sticks that move a LOITER copter with ENU heading ``yaw`` toward the ENU direction (east, north)."""
        fwd = east * math.cos(yaw) + north * math.sin(yaw)
        right = east * math.sin(yaw) - north * math.cos(yaw)
        self.sticks = (1500 + amount * right, 1500 - amount * fwd, 1500, 1500)

    def set_mode(self, mode):
        if not self.c_mode.wait_for_service(timeout_sec=5):
            return False
        r = self.SetMode.Request()
        r.custom_mode = mode
        f = self.c_mode.call_async(r)
        t0 = time.time()
        while not f.done() and time.time() - t0 < 5:
            time.sleep(0.02)
        return bool(f.done() and f.result() and f.result().mode_sent)

    def goto(self, x, y, z):
        m = self.PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        self.p_sp.publish(m)


class Square:
    """The square demo as a separate process, like the desktop icon starts it."""

    def __init__(self, *args):
        env = dict(os.environ)
        for k in ("CYCLONEDDS_URI", "ROS_DOMAIN_ID"):
            env.pop(k, None)
        repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.p = subprocess.Popen([sys.executable, "-m", "dronelab.demos.square", "--drone", "1", "--yes", *args],
                                  cwd=repo, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.lines = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            self.lines.append(line.rstrip())
            print("    square| " + line.rstrip(), flush=True)

    def saw(self, text):
        return any(text in ln for ln in self.lines)

    def wait(self, timeout):
        try:
            return self.p.wait(timeout)
        except subprocess.TimeoutExpired:
            self.p.terminate()
            return None


def private_participant_sees_mavros(domain) -> bool:
    """A default (network) participant on the private domain must NOT discover MAVROS."""
    import subprocess
    code = ("import rclpy,time\nfrom rclpy.node import Node\nrclpy.init()\nn=Node('intruder')\n"
            "t=time.time()\nwhile time.time()-t<6: rclpy.spin_once(n,timeout_sec=0.2)\n"
            "print(any(x.startswith('/mavros') for x,_ in n.get_topic_names_and_types()))")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    env.pop("CYCLONEDDS_URI", None)
    out = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=30).stdout
    return "True" in out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--attach", action="store_true")
    a = ap.parse_args()
    cfg = Config.load()
    ds = cfg.drone(1)
    stack = None
    if not a.attach:
        from dronelab.sim.stack import SimStack
        stack = SimStack(cfg)
        stack.start(show_gazebo=False)
    s = Student(int((cfg.data.get("demo") or {}).get("student_domain", 0)))
    try:
        ok = s.wait(lambda: s.state is not None and s.state.connected, 180, 0.5)
        if not check("MAVROS connected through the gate (/drone1/state)", ok):
            return
        check("FPV camera publishing", s.wait(lambda: s.images > 3, 30))
        check("isolation: network participant on private domain cannot see MAVROS",
              not private_participant_sees_mavros(ds.private_domain))
        admin = AdminClient(ds.socket_path)

        def status():
            admin.poll()
            return admin.last_status or {}
        ok = s.wait(lambda: status().get("params_checked"), 60, 1.0)
        st = status()
        check("gate read the ArduPilot parameters", ok, f"({len(st.get('params') or {})} values, "
              f"{len(st.get('param_warnings') or [])} warnings)")
        check("parameter review: RC_OPTIONS overrides flagged (SITL default 0)",
              any("RC_OPTIONS" in w for w in st.get("param_warnings") or []))

        # ---- the square demo from the pad (arms, takes off, 3 x 3 m, lands)
        track = []
        sampling = threading.Event()

        def sample():
            while not sampling.is_set():
                if s.pose:
                    track.append(s.pose)
                time.sleep(0.1)
        threading.Thread(target=sample, daemon=True).start()
        t0 = time.time()
        sq = Square()
        rc = sq.wait(240)
        sampling.set()
        xs, ys, zs = [p[0] for p in track], [p[1] for p in track], [p[2] for p in track]
        check("square demo: took off, flew, landed (exit 0)", rc == 0 and sq.saw("landed and disarmed"),
              f"(exit {rc}, {time.time() - t0:.0f} s)")
        if track:
            check("square demo: 3 x 3 m at 3 m", 2.6 <= max(xs) - min(xs) <= 3.5 and 2.6 <= max(ys) - min(ys) <= 3.5
                  and 2.6 <= max(zs) <= 3.5,
                  f"(x {min(xs):.1f}..{max(xs):.1f}, y {min(ys):.1f}..{max(ys):.1f}, max z {max(zs):.1f})")
        s.wait(lambda: s.state and not s.state.armed, 30, 0.5)
        # ---- arm + takeoff via the student API (EKF needs up to ~60 s after SITL start)
        t0 = time.time()
        armed = False
        while time.time() - t0 < 120 and not armed:
            s.mode("GUIDED")
            armed = s.arm(True)
            if not armed:
                time.sleep(2)
        if not check("student: GUIDED + cmd/arming", armed, f"({time.time() - t0:.0f} s)"):
            return
        check("student: cmd/takeoff 3 m", s.takeoff(3.0))
        check("climbed to 3 m", s.wait(lambda: s.pose and s.pose[2] > 2.6, 30))
        check("takeoff above the fence ceiling is rejected", not s.takeoff(20.0))

        # ---- climb above the trees/low buildings into a clear lane, then fly east into the fence
        t0 = time.time()
        while time.time() - t0 < 40 and not (s.pose and math.dist(s.pose, (20.0, -20.0, 12.0)) < 1.0):
            s.goto(20.0, -20.0, 12.0)
            time.sleep(0.2)
        check("position setpoint inside the fence is flown", math.dist(s.pose, (20.0, -20.0, 12.0)) < 1.5,
              f"(at {s.pose[0]:.1f}, {s.pose[1]:.1f}, {s.pose[2]:.1f})")
        # ---- velocity toward the east wall (x max 28) must stop before it
        tt = time.monotonic()
        maxx = -1e9
        t0 = time.time()
        while time.time() - t0 < 16:
            s.vel_cmd(6.0, 0.0)
            maxx = max(maxx, s.pose[0])
            time.sleep(0.05)
        check("velocity: stopped at the fence", maxx <= 28.0 + 0.6, f"(max x = {maxx:.2f}, wall 28)")
        check("velocity: limitation reported on /drone1/error", bool(s.errors_since(tt, "slowed")))

        # ---- stop streaming: the gate must hover within ~0.5 s (ArduPilot alone would fly on for 3 s)
        for _ in range(40):
            s.vel_cmd(-4.0, 0.0)
            time.sleep(0.05)
        tt = time.monotonic()
        x_stop = s.pose[0]
        time.sleep(2.5)
        speed = math.hypot(*s.vel[:2])
        check("watchdog: hover after the velocity stream stops", bool(s.errors_since(tt, "hovering")) and speed < 0.6,
              f"(speed {speed:.2f} m/s, drifted {abs(s.pose[0] - x_stop):.1f} m)")

        # ---- position outside the fence is rejected
        tt = time.monotonic()
        p0 = s.pose
        for _ in range(10):
            s.goto(s.pose[0], 60.0, 12.0)
            time.sleep(0.1)
        time.sleep(2)
        check("position outside fence: rejected + error", bool(s.errors_since(tt, "outside fence"))
              and abs(s.pose[1] - p0[1]) < 2.0)
        # ---- position + feed-forward velocity: velocity part removed
        tt = time.monotonic()
        pt = s.m["PositionTarget"]()
        pt.coordinate_frame = 1
        pt.type_mask = 64 | 128 | 256 | 1024 | 2048
        pt.position.x, pt.position.y, pt.position.z = 20.0, -20.0, 12.0
        pt.velocity.x = 3.0
        for _ in range(5):
            s.p_raw.publish(pt)
            time.sleep(0.1)
        time.sleep(0.5)
        check("raw position+velocity: feed-forward removed", bool(s.errors_since(tt, "feed-forward")))
        check("forbidden mode (ACRO) rejected", not s.mode("ACRO"))
        check("LOITER is the pilot's mode: students cannot select it", not s.mode("LOITER"))

        # ---- RC pilot takeover during the square demo (in the air, at 12 m)
        pv = Private(ds.private_domain)
        time.sleep(2)
        sq = Square("--alt", "12", "--no-land", "--hold", "0.5")
        reached = s.wait(lambda: sq.saw("corner 3/4") or sq.p.poll() is not None, 90, 0.2)
        check("square demo started in the air", reached and sq.p.poll() is None)
        pv.sticks = (1500, 1500, 1500, 1500)              # sticks centred, throttle mid = hold altitude
        time.sleep(0.3)
        took = pv.set_mode("LOITER") and s.wait(lambda: s.state.mode == "LOITER", 5)
        check("pilot: mode switch to LOITER", took)
        rc = sq.wait(5)
        check("square demo stops at once when the pilot takes over", rc == 2 and
              (sq.saw("left GUIDED") or sq.saw("took over")), f"(exit {rc})")
        ok = s.wait(lambda: status().get("pilot_control") == "LOITER", 3, 0.2)
        check("gate status: pilot_control = LOITER", ok)
        tt = time.monotonic()
        check("pilot control: student GUIDED rejected", not s.mode("GUIDED"))
        check("pilot control: student land rejected", not s.land())
        for _ in range(10):
            s.vel_cmd(0.0, 3.0)
            time.sleep(0.05)
        check("pilot control: student velocity rejected with an error",
              bool(s.errors_since(tt, "RC pilot")) and s.state.mode == "LOITER")
        # the pilot flies out of the fence: the gate must NOT land or lock
        t0 = time.time()
        maxx = s.pose[0]
        while time.time() - t0 < 25 and s.pose[0] < 33.0:
            pv.stick_toward(s.yaw, 1.0, 0.0)
            maxx = max(maxx, s.pose[0])
            time.sleep(0.1)
        pv.sticks = (1500, 1500, 1500, 1500)
        time.sleep(2.0)
        st = status()
        check("pilot flies outside the fence: no LAND, no lock", s.pose[0] > 30.0 and s.state.mode == "LOITER"
              and not st.get("locked"), f"(x = {s.pose[0]:.1f}, fence 28 + margin 2, mode {s.state.mode})")
        t0 = time.time()
        while time.time() - t0 < 25 and s.pose[0] > 22.0:
            pv.stick_toward(s.yaw, -1.0, 0.0)
            time.sleep(0.1)
        pv.sticks = (1500, 1500, 1500, 1500)
        s.wait(lambda: math.hypot(*s.vel[:2]) < 0.4, 15)
        admin.send("mode", mode="GUIDED")                  # instructor hands the drone back to the students
        back = s.wait(lambda: s.state.mode == "GUIDED", 8)
        pv.sticks = (0, 0, 0, 0)                           # release the override
        check("hand back: GUIDED from the Flight app, pilot_control cleared",
              back and s.wait(lambda: status().get("pilot_control") is None, 3, 0.2))
        t0 = time.time()
        while time.time() - t0 < 30 and math.dist(s.pose, (20.0, -20.0, 12.0)) > 1.0:
            s.goto(20.0, -20.0, 12.0)
            time.sleep(0.2)
        check("students fly again after the hand-back", math.dist(s.pose, (20.0, -20.0, 12.0)) < 1.5)

        # ---- breach: a program bypassing the gate flies out in GUIDED -> the gate lands + locks
        breach_ok = False
        tt = time.monotonic()
        t0 = time.time()
        while time.time() - t0 < 40:
            if s.state and s.state.mode == "LAND":
                breach_ok = True
                break
            pv.goto(45.0, -20.0, 12.0)
            time.sleep(0.1)
        check("breach: drone outside the fence -> LAND", breach_ok,
              f"(x = {s.pose[0]:.1f}, fence 28 + margin 2)")
        check("breach: error published", bool(s.errors_since(tt, "BREACH")))
        check("breach: student cannot switch back to GUIDED", not s.mode("GUIDED"))
        admin.send("release_lock")
        time.sleep(1)
        st = admin.poll() or admin.last_status
        check("admin: lock released", st and not st.get("locked"))
        ok = s.wait(lambda: s.state and not s.state.armed, 90, 0.5)
        check("landed and disarmed", ok)
        check("extended_state streamed (landed state known)", s.landed in (1, 2, 3, 4), f"(landed_state={s.landed})")

        # ---- "Fix parameters" from the Flight app (only on the ground)
        cid = admin.send("fix_params")
        reply = None
        t0 = time.time()
        while time.time() - t0 < 60 and reply is None:
            admin.poll()
            reply = next((r for r in admin.pop_replies() if r.get("id") == cid), None)
            time.sleep(0.2)
        check("fix_params written", reply and reply.get("ok"), f"({(reply or {}).get('msg', 'no reply')[:160]})")
        time.sleep(3)
        st = status()
        p = st.get("params") or {}
        check("after the fix: RC overrides ignored, GCS failsafe on, no warnings",
              int(p.get("RC_OPTIONS", 0)) & 2 and int(p.get("FS_GCS_ENABLE", 0)) == 5
              and int(p.get("FS_OPTIONS", 0)) & 16 and not st.get("param_warnings"),
              f"(RC_OPTIONS={p.get('RC_OPTIONS')}, FS_GCS_ENABLE={p.get('FS_GCS_ENABLE')}, "
              f"warnings={st.get('param_warnings')})")
    finally:
        print("\n=== SUMMARY ===")
        for name, ok, d in RESULTS:
            print(f"{'PASS' if ok else 'FAIL'}  {name} {d}")
        n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
        print(f"{len(RESULTS) - n_fail}/{len(RESULTS)} passed")
        if stack:
            stack.stop()
        os._exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
