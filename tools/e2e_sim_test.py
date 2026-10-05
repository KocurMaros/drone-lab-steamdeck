#!/usr/bin/env python3
"""End-to-end self test of the simulation + safety gate (run inside the container: DroneLab Shell).

    python3 tools/e2e_sim_test.py            # starts Gazebo (headless) + SITL + gate, runs checks, stops
    python3 tools/e2e_sim_test.py --attach   # use a simulation that is already running (e.g. the Demo app)

Flies the simulated drone through the STUDENT interface (/drone1/... on the student domain) and checks
that the gate limits, rejects, hovers, and lands + locks on a fence breach.
"""
from __future__ import annotations

import argparse
import math
import os
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

        # ---- breach: fly out by talking to MAVROS directly (bypassing the gate, like an RC pilot would)
        admin = AdminClient(ds.socket_path)
        breach_ok = False
        try:
            xml = write_cyclone_config("e2e-private", [ds.private_domain], None)
            os.environ.update(ros_env(ds.private_domain, xml))
            from rclpy.context import Context
            from rclpy.node import Node
            from geometry_msgs.msg import PoseStamped
            ctx = Context()
            ctx.init(domain_id=ds.private_domain)
            pn = Node("e2e_bypass", context=ctx)
            pub = pn.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
            time.sleep(2)
            tt = time.monotonic()
            t0 = time.time()
            while time.time() - t0 < 40:
                if s.state and s.state.mode == "LAND":
                    breach_ok = True
                    break
                m = PoseStamped()
                m.header.frame_id = "map"
                m.pose.position.x, m.pose.position.y, m.pose.position.z = 45.0, -20.0, 12.0
                m.pose.orientation.w = 1.0
                pub.publish(m)
                time.sleep(0.1)
            pn.destroy_node()
            ctx.try_shutdown()
        except Exception as e:
            print("bypass publisher failed:", e)
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
