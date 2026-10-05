"""The student gate: one process per drone.

    students (domain 0, lab network)          gate            MAVROS (private domain, loopback only)
    /droneNN/setpoint_* , services   --->  validate/limit  --->  /mavros/setpoint_* , services
    /droneNN/<every MAVROS topic>     <---  relay          <---  /mavros/<topic>
    /droneNN/error, /droneNN/gate/status   (published by the gate)

rclpy is imported only after the DDS environment has been prepared by
`dronelab.gate.__main__`.
"""
from __future__ import annotations

import collections
import functools
import json
import math
import threading
import time
import traceback
from typing import Dict, Optional, Tuple

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)

from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import ExtendedState, GlobalPositionTarget, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, MessageInterval, SetMode
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

from .. import geo
from ..config import DroneSettings
from ..geo import Vec3
from ..safety import (ACC_BITS, FORCE, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED, FRAME_GLOBAL_REL_ALT,
                      FRAME_GLOBAL_REL_ALT_INT, FRAME_LOCAL_NED, IGNORE_YAW, IGNORE_YAW_RATE, POS_BITS, VEL_BITS,
                      CommandGate, Decision, VehicleState)

GATE_NODE = "dronelab_gate"
TRANSFORMED = ("local_position/pose", "local_position/odom", "local_position/velocity_local",
               "setpoint_raw/target_local")
SKIP_RELAY_SUFFIX = ("/parameter_events", "/rosout")
# MAVLink messages the gate needs at a usable rate (ArduPilot's default stream rates are 2-4 Hz
# and EXTENDED_SYS_STATE is not streamed at all unless requested)
STREAMS = {32: 20.0,    # LOCAL_POSITION_NED
           30: 20.0,    # ATTITUDE
           33: 10.0,    # GLOBAL_POSITION_INT
           245: 4.0}    # EXTENDED_SYS_STATE (landed state)
VEL_TIMEOUT_S = 0.5     # stop a velocity the student stopped sending (ArduPilot would keep it 3 s)

STATE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _quat_ok(q) -> bool:
    n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    return math.isfinite(n) and n > 0.25


def guarded(fn):
    """Never let an exception escape a callback: in rclpy Humble it would kill the executor."""
    @functools.wraps(fn)
    def wrapper(self, *a, **kw):
        try:
            return fn(self, *a, **kw)
        except Exception as e:
            self.callback_errors += 1
            self.log(f"[gate] callback {fn.__name__} failed: {e}\n{traceback.format_exc()}")
            if a and hasattr(a[-1], "success"):       # service response object
                a[-1].success = False
                return a[-1]
            if a and hasattr(a[-1], "mode_sent"):
                a[-1].mode_sent = False
                return a[-1]
            return None
    return wrapper


class Gate:
    def __init__(self, ds: DroneSettings, mavros_ns: str = "/mavros", log=print):
        self.ds = ds
        self.mns = mavros_ns.rstrip("/")
        self.sns = ds.student_ns
        self.log = log
        self.callback_errors = 0
        self.lock = threading.RLock()
        self.vs = VehicleState()
        self.cg = CommandGate(ds.fence, ds.limits, ds.rules)
        self.a2l = ds.arena_to_local
        self.l2a = ds.arena_to_local.inverse()
        self.is_global = ds.fence.kind == "global"
        self.local_pose: Optional[Tuple[Vec3, float, float]] = None   # local ENU pos, yaw, t
        self.global_fix: Optional[Tuple[float, float, float]] = None  # lat, lon, t
        self.rel_alt: Optional[Tuple[float, float]] = None            # rel alt, t
        self.plane_offset: Optional[Vec3] = None                      # fence plane - local ENU (low-passed)
        self.battery: Dict = {}
        self.errors = collections.deque(maxlen=40)
        self._err_last: Dict[str, float] = {}
        self.student_activity: Dict[str, float] = {}
        self.last_breach = ""
        self.started = time.monotonic()
        self.ext_state_seen = 0.0
        self.pose_times = collections.deque(maxlen=64)
        self._streams_requested = 0.0
        self._last_vel_t = 0.0          # last NON-ZERO velocity forwarded to MAVROS
        self._seq_lock = threading.Lock()           # one arming sequence at a time
        self._svc_sem = threading.BoundedSemaphore(3)  # blocking student services in flight
        self._breach_thread: Optional[threading.Thread] = None

        # ---------- two isolated ROS contexts ----------
        self.ctx_p = Context()
        self.ctx_p.init(domain_id=ds.private_domain)
        self.ctx_s = Context()
        self.ctx_s.init(domain_id=ds.student_domain)
        self.np = Node(GATE_NODE, context=self.ctx_p)
        # students must not be able to change the gate node's parameters (e.g. use_sim_time)
        self.ns = Node(f"{GATE_NODE}_{ds.sysid}", context=self.ctx_s, start_parameter_services=False)
        self.cg_state = MutuallyExclusiveCallbackGroup()
        self.cg_relay = MutuallyExclusiveCallbackGroup()
        self.cg_scan = MutuallyExclusiveCallbackGroup()
        self.cg_timer = MutuallyExclusiveCallbackGroup()
        self.cg_cli = ReentrantCallbackGroup()
        self.cg_cmd = MutuallyExclusiveCallbackGroup()
        self.cg_srv = ReentrantCallbackGroup()
        self.cg_land = MutuallyExclusiveCallbackGroup()

        self._setup_private()
        self._setup_student()
        self.relays: Dict[str, tuple] = {}
        self.np.create_timer(2.0, self._scan_relays, callback_group=self.cg_scan)
        self.np.create_timer(0.1, self._watchdog, callback_group=self.cg_timer)
        self.ns.create_timer(1.0 / max(ds.status_rate_hz, 0.1), self._publish_status)
        self.ns.create_timer(2.0, self._publish_fence)

        self.ex_p = MultiThreadedExecutor(num_threads=4, context=self.ctx_p)
        self.ex_p.add_node(self.np)
        self.ex_s = MultiThreadedExecutor(num_threads=8, context=self.ctx_s)
        self.ex_s.add_node(self.ns)
        self.threads = [threading.Thread(target=self._spin, args=(self.ex_p,), daemon=True, name="spin-private"),
                        threading.Thread(target=self._spin, args=(self.ex_s,), daemon=True, name="spin-student")]
        for t in self.threads:
            t.start()
        self.log(f"[gate] drone {ds.sysid}: students on domain {ds.student_domain} {self.sns}/..., "
                 f"MAVROS on private domain {ds.private_domain} {self.mns}/... (loopback only)")

    def _spin(self, ex):
        try:
            ex.spin()
        except Exception as e:
            self.log(f"[gate] executor stopped: {e}\n{traceback.format_exc()}")

    @property
    def executors_alive(self) -> bool:
        return all(t.is_alive() for t in self.threads)

    def shutdown(self):
        for ex in (self.ex_s, self.ex_p):
            try:
                ex.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        for n in (self.ns, self.np):
            try:
                n.destroy_node()
            except Exception:
                pass
        for c in (self.ctx_s, self.ctx_p):
            try:
                c.try_shutdown()
            except Exception:
                pass

    # =================================================================== private side
    def _setup_private(self):
        n, m = self.np, self.mns
        n.create_subscription(State, f"{m}/state", self._on_state, STATE_QOS, callback_group=self.cg_state)
        n.create_subscription(ExtendedState, f"{m}/extended_state", self._on_ext_state, STATE_QOS,
                              callback_group=self.cg_state)
        n.create_subscription(PoseStamped, f"{m}/local_position/pose", self._on_local_pose, qos_profile_sensor_data,
                              callback_group=self.cg_state)
        n.create_subscription(NavSatFix, f"{m}/global_position/global", self._on_global, qos_profile_sensor_data,
                              callback_group=self.cg_state)
        n.create_subscription(Float64, f"{m}/global_position/rel_alt", self._on_rel_alt, qos_profile_sensor_data,
                              callback_group=self.cg_state)
        n.create_subscription(BatteryState, f"{m}/battery", self._on_battery, qos_profile_sensor_data,
                              callback_group=self.cg_state)
        self.p_sp_local = n.create_publisher(PoseStamped, f"{m}/setpoint_position/local", 10)
        self.p_sp_vel = n.create_publisher(Twist, f"{m}/setpoint_velocity/cmd_vel_unstamped", 10)
        self.p_sp_vel_st = n.create_publisher(TwistStamped, f"{m}/setpoint_velocity/cmd_vel", 10)
        self.p_raw_local = n.create_publisher(PositionTarget, f"{m}/setpoint_raw/local", 10)
        self.p_raw_global = n.create_publisher(GlobalPositionTarget, f"{m}/setpoint_raw/global", 10)
        self.c_mode = n.create_client(SetMode, f"{m}/set_mode", callback_group=self.cg_cli)
        self.c_arm = n.create_client(CommandBool, f"{m}/cmd/arming", callback_group=self.cg_cli)
        self.c_takeoff = n.create_client(CommandTOL, f"{m}/cmd/takeoff", callback_group=self.cg_cli)
        self.c_land = n.create_client(CommandTOL, f"{m}/cmd/land", callback_group=self.cg_cli)
        self.c_cmd = n.create_client(CommandLong, f"{m}/cmd/command", callback_group=self.cg_cli)
        self.c_interval = n.create_client(MessageInterval, f"{m}/set_message_interval", callback_group=self.cg_cli)

    @guarded
    def _on_state(self, msg: State):
        with self.lock:
            was = self.vs.connected
            self.vs.armed = msg.armed
            self.vs.mode = msg.mode
            self.vs.connected = msg.connected
        if msg.connected and not was:
            self._streams_requested = 0.0   # (re)connected: ask for our stream rates again

    @guarded
    def _on_ext_state(self, msg: ExtendedState):
        with self.lock:
            self.vs.landed_state = msg.landed_state
            self.ext_state_seen = time.monotonic()

    @guarded
    def _on_battery(self, msg: BatteryState):
        with self.lock:
            self.battery = {"voltage": round(msg.voltage, 2),
                            "percentage": round(msg.percentage * 100.0, 1) if msg.percentage >= 0 else None}

    @guarded
    def _on_local_pose(self, msg: PoseStamped):
        now = time.monotonic()
        p = Vec3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        q = msg.pose.orientation
        yaw = geo.yaw_from_quat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.local_pose = (p, yaw, now)
            self.pose_times.append(now)
            if self.is_global:
                self.vs.yaw = yaw
            else:
                self.vs.pos = self.l2a.point(p)
                self.vs.yaw = self.l2a.heading(yaw)
                self.vs.pos_stamp = now
        if not self.is_global:
            self._check_breach()

    @guarded
    def _on_global(self, msg: NavSatFix):
        if msg.status.status < 0 or not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        now = time.monotonic()
        with self.lock:
            self.global_fix = (msg.latitude, msg.longitude, now)
            if self.is_global and self.rel_alt and now - self.rel_alt[1] < self.ds.limits.pose_timeout_s:
                e, n = self.ds.fence.to_plane(msg.latitude, msg.longitude)
                self.vs.pos = Vec3(e, n, self.rel_alt[0])
                self.vs.pos_stamp = now
                lp = self.local_pose
                if lp and now - lp[2] < 0.15:   # only pair samples taken close together
                    off = self.vs.pos - lp[0]
                    po = self.plane_offset
                    self.plane_offset = off if po is None else Vec3(po.x + 0.1 * (off.x - po.x),
                                                                    po.y + 0.1 * (off.y - po.y),
                                                                    po.z + 0.1 * (off.z - po.z))
        if self.is_global:
            self._check_breach()

    @guarded
    def _on_rel_alt(self, msg: Float64):
        with self.lock:
            self.rel_alt = (msg.data, time.monotonic())

    def _check_breach(self):
        with self.lock:
            why = self.cg.update_breach(self.vs)
            action = self.ds.rules.breach_action
        if not why:
            return
        self.last_breach = why
        if action == "none":
            self._error(f"FENCE BREACH: {why}", key="breach")
            return
        mode = "LAND" if action == "land" else "BRAKE"
        self._enforce_mode(mode, f"fence breach: {why}")

    def _enforce_mode(self, mode: str, why: str):
        """Send the mode first, then tell everyone; retry until the drone reports it (handles a student
        request racing with us), fall back from BRAKE to LAND. Never repeats once reached, so an
        instructor taking over with the RC afterwards is not fought."""
        def run():
            target = mode
            for attempt in range(8):
                if self.c_mode.service_is_ready():
                    req = SetMode.Request()
                    req.custom_mode = target
                    self.c_mode.call_async(req)
                t0 = time.monotonic()
                while time.monotonic() - t0 < 1.0:
                    with self.lock:
                        if self.vs.mode == target or not self.vs.armed:
                            self.log(f"[gate] {target} confirmed ({why})")
                            return
                    time.sleep(0.05)
                if target == "BRAKE" and attempt >= 1:
                    target = "LAND"
            self._error(f"COULD NOT SWITCH TO {target} after {why} - take over with the transmitter!",
                        key="enforce_fail", force=True)
        self._breach_thread = threading.Thread(target=run, daemon=True, name="enforce-mode")
        self._breach_thread.start()
        self._error(f"FENCE BREACH: {why.split(': ', 1)[-1]} -> switching to {mode}; student commands locked "
                    f"until the instructor releases the lock" if why.startswith("fence") else
                    f"{why} -> {mode}", key="breach", force=True)

    # =================================================================== watchdog (10 Hz, private executor)
    @guarded
    def _watchdog(self):
        now = time.monotonic()
        with self.lock:
            connected, armed, in_air = self.vs.connected, self.vs.armed, self.vs.in_air
            stale = not self.vs.pos_fresh(1.0, now)
            last_vel = self._last_vel_t
        # 1) the student stopped streaming a velocity: hover now instead of ArduPilot's 3 s timeout
        if last_vel and now - last_vel > VEL_TIMEOUT_S:
            with self.lock:
                self._last_vel_t = 0.0
            self._publish_hover()
            self._error(f"no velocity command for {VEL_TIMEOUT_S} s: hovering", key="vel_timeout")
        # 2) stream rates: ask for them after connecting and whenever the pose rate is too low
        if connected and now - self._streams_requested > 5.0:
            with self.lock:
                rate = sum(1 for t in self.pose_times if now - t < 1.0)
            if self._streams_requested == 0.0 or rate < 10 or now - self.ext_state_seen > 3.0:
                if self._request_streams():
                    self._streams_requested = now
            else:
                self._streams_requested = now
        # 3) position lost while flying: the breach monitor is blind - say so loudly
        if armed and in_air and stale:
            self._error("NO POSITION from the drone for >1 s while flying - fence cannot be checked, "
                        "student velocity commands are replaced by hover", key="pose_lost")

    def _request_streams(self) -> bool:
        if not self.c_interval.service_is_ready():
            return False
        for mid, hz in STREAMS.items():
            req = MessageInterval.Request()
            req.message_id = mid
            req.message_rate = float(hz)
            self.c_interval.call_async(req)
        return True

    def _publish_hover(self):
        out = PositionTarget()
        out.header.stamp = self._now_msg()
        out.header.frame_id = "map"
        out.coordinate_frame = FRAME_LOCAL_NED
        out.type_mask = POS_BITS | ACC_BITS | IGNORE_YAW
        self.p_raw_local.publish(out)

    def _note_velocity(self, v: Vec3):
        with self.lock:
            moving = abs(v.x) + abs(v.y) + abs(v.z) > 1e-3
            self._last_vel_t = time.monotonic() if moving else 0.0

    # =================================================================== relays
    @guarded
    def _scan_relays(self):
        topics = self.np.get_topic_names_and_types()
        for name, types in topics:
            with self.lock:
                known = name in self.relays
            if not name.startswith(self.mns + "/") or known or not types:
                continue
            if name.endswith(SKIP_RELAY_SUFFIX):
                continue
            pubs = [i for i in self.np.get_publishers_info_by_topic(name) if i.node_name != GATE_NODE]
            if not pubs:
                continue  # topic only subscribed by MAVROS (a command input): never relayed back
            rel = (ReliabilityPolicy.BEST_EFFORT
                   if any(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for p in pubs)
                   else ReliabilityPolicy.RELIABLE)
            dur = (DurabilityPolicy.TRANSIENT_LOCAL
                   if all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in pubs)
                   else DurabilityPolicy.VOLATILE)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=rel, durability=dur)
            suffix = name[len(self.mns) + 1:]
            out = f"{self.sns}/{suffix}"
            try:
                from rosidl_runtime_py.utilities import get_message
                cls = get_message(types[0])
            except Exception as e:
                self.log(f"[gate] cannot relay {name} ({types[0]}): {e}")
                with self.lock:
                    self.relays[name] = None
                continue
            pub = self.ns.create_publisher(cls, out, qos)
            if suffix in TRANSFORMED and not self.a2l.is_identity:
                cb = self._make_transform_relay(suffix, pub)
                sub = self.np.create_subscription(cls, name, cb, qos, callback_group=self.cg_relay)
            else:
                sub = self.np.create_subscription(cls, name, self._make_raw_relay(pub), qos,
                                                  callback_group=self.cg_relay, raw=True)
            with self.lock:
                self.relays[name] = (sub, pub)

    def _make_raw_relay(self, pub):
        def cb(raw):
            try:
                pub.publish(raw)
            except Exception as e:  # never kill the executor because of one topic
                self.callback_errors += 1
                self.log(f"[gate] relay to {pub.topic_name} failed: {e}")
        return cb

    def _make_transform_relay(self, suffix, pub):
        l2a = self.l2a

        def tf_pose(pose):
            p = l2a.point(Vec3(pose.position.x, pose.position.y, pose.position.z))
            q = pose.orientation
            # rotate orientation by the frame yaw (z-axis rotation composes on the left)
            half = l2a.yaw / 2.0
            cz, sz = math.cos(half), math.sin(half)
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            q.w, q.x, q.y, q.z = (cz * qw - sz * qz, cz * qx - sz * qy, cz * qy + sz * qx, cz * qz + sz * qw)
            pose.position.x, pose.position.y, pose.position.z = float(p.x), float(p.y), float(p.z)

        def tf_vec(v):
            r = l2a.vector(Vec3(v.x, v.y, v.z))
            v.x, v.y, v.z = float(r.x), float(r.y), float(r.z)

        def convert(msg):
            if suffix == "local_position/pose":
                tf_pose(msg.pose)
            elif suffix == "local_position/odom":
                tf_pose(msg.pose.pose)   # Odometry twist is in the child (body) frame: unchanged
            elif suffix == "local_position/velocity_local":
                tf_vec(msg.twist.linear)
            elif msg.coordinate_frame == FRAME_LOCAL_NED:  # setpoint_raw/target_local
                p = l2a.point(Vec3(msg.position.x, msg.position.y, msg.position.z))
                msg.position.x, msg.position.y, msg.position.z = float(p.x), float(p.y), float(p.z)
                tf_vec(msg.velocity)
                tf_vec(msg.acceleration_or_force)
                msg.yaw = float(l2a.heading(msg.yaw))

        def cb(msg):
            try:
                convert(msg)
                pub.publish(msg)
            except Exception as e:
                self.callback_errors += 1
                self.log(f"[gate] relay {suffix} failed: {e}")
        return cb

    # =================================================================== student side
    def _setup_student(self):
        n, s = self.ns, self.sns
        q = qos_profile_sensor_data  # best effort matches reliable AND best-effort student publishers
        n.create_subscription(PoseStamped, f"{s}/setpoint_position/local", self._on_sp_local, q,
                              callback_group=self.cg_cmd)
        n.create_subscription(Twist, f"{s}/setpoint_velocity/cmd_vel_unstamped", self._on_sp_vel, q,
                              callback_group=self.cg_cmd)
        n.create_subscription(TwistStamped, f"{s}/setpoint_velocity/cmd_vel", self._on_sp_vel_stamped, q,
                              callback_group=self.cg_cmd)
        n.create_subscription(PositionTarget, f"{s}/setpoint_raw/local", self._on_raw_local, q,
                              callback_group=self.cg_cmd)
        n.create_subscription(GeoPoseStamped, f"{s}/setpoint_position/global", self._on_sp_global, q,
                              callback_group=self.cg_cmd)
        n.create_subscription(GlobalPositionTarget, f"{s}/setpoint_raw/global", self._on_raw_global, q,
                              callback_group=self.cg_cmd)
        n.create_service(CommandBool, f"{s}/cmd/arming", self._srv_arming, callback_group=self.cg_srv)
        n.create_service(SetMode, f"{s}/set_mode", self._srv_mode, callback_group=self.cg_srv)
        n.create_service(CommandTOL, f"{s}/cmd/takeoff", self._srv_takeoff, callback_group=self.cg_srv)
        n.create_service(CommandTOL, f"{s}/cmd/land", self._srv_land, callback_group=self.cg_land)
        self.p_err = n.create_publisher(String, f"{s}/error", 10)
        self.p_status = n.create_publisher(String, f"{s}/gate/status", LATCHED)
        self.p_fence = n.create_publisher(Marker, f"{s}/gate/fence", LATCHED)

    # ----- helpers -----
    def _error(self, text: str, key: str = "", force: bool = False):
        now = time.monotonic()
        key = key or text[:40]
        with self.lock:
            last = self._err_last.get(key, 0.0)
            if self.errors and self.errors[-1]["text"] == text:
                self.errors[-1]["count"] += 1
                self.errors[-1]["t"] = time.time()
            else:
                self.errors.append({"t": time.time(), "text": text, "count": 1})
            if not force and now - last < 1.0 / max(self.ds.error_rate_hz, 0.01):
                return
            self._err_last[key] = now
        try:
            self.p_err.publish(String(data=text))
            self.log(f"[gate] {text}")
        except Exception:
            pass

    def _reject(self, d: Decision, what: str):
        self._error(f"{what}: {d.reason}", key=f"{what}:{d.key}")

    def _touch(self, what: str):
        with self.lock:
            self.student_activity[what] = time.monotonic()

    def _now_msg(self):
        return self.np.get_clock().now().to_msg()

    def _snapshot(self) -> VehicleState:
        with self.lock:
            v = self.vs
            return VehicleState(pos=v.pos, yaw=v.yaw, pos_stamp=v.pos_stamp, armed=v.armed, mode=v.mode,
                                connected=v.connected, landed_state=v.landed_state)

    def _local_to_fence(self, p_local: Vec3) -> Optional[Vec3]:
        """Local ENU point -> fence coordinates. Outdoors via the low-passed local->plane offset."""
        if not self.is_global:
            return self.l2a.point(p_local)
        with self.lock:
            off = self.plane_offset
            fresh = self.vs.pos_fresh(self.ds.limits.pose_timeout_s)
        if off is None or not fresh:
            return None
        return p_local + off

    def _yaw_or_current(self, q) -> float:
        """Student heading in the arena frame; an all-zero quaternion means 'keep current heading'."""
        if _quat_ok(q):
            return geo.yaw_from_quat(q.x, q.y, q.z, q.w)
        with self.lock:
            return self.vs.yaw if self.vs.yaw is not None else 0.0

    # ----- setpoints -----
    @guarded
    def _on_sp_local(self, msg: PoseStamped):
        self._touch("setpoint_position/local")
        p_arena = Vec3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        yaw_arena = self._yaw_or_current(msg.pose.orientation)
        p_local = self.a2l.point(p_arena) if not self.is_global else p_arena
        fence_pt = p_arena if not self.is_global else self._local_to_fence(p_local)
        if fence_pt is None:
            self._error("setpoint_position/local: no fresh local+GPS position to check it against the outdoor "
                        "fence", key="sp_local_nopos")
            return
        d = self.cg.check_position(fence_pt, self._snapshot())
        if not d.ok:
            return self._reject(d, "setpoint_position/local")
        out = PoseStamped()
        out.header.stamp = self._now_msg()
        out.header.frame_id = "map"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = float(p_local.x), float(p_local.y), float(p_local.z)
        yaw_local = self.a2l.heading(yaw_arena) if not self.is_global else yaw_arena
        qx, qy, qz, qw = geo.quat_from_yaw(yaw_local)
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = float(qx), float(qy), float(qz), float(qw)
        self.p_sp_local.publish(out)
        self._note_velocity(Vec3(0, 0, 0))

    def _velocity(self, lin, yaw_rate: float, what: str) -> Optional[Tuple[Vec3, float, bool]]:
        v_arena = Vec3(lin.x, lin.y, lin.z)
        d = self.cg.check_velocity(v_arena, yaw_rate, self._snapshot())
        if not d.ok:
            self._reject(d, what)
        elif d.modified:
            self._error(f"{what}: {d.reason}", key=f"{what}:{d.key}")
        if d.value is None:
            return None
        v, yr = d.value
        v_local = self.a2l.vector(v) if not self.is_global else v
        self._note_velocity(v_local)
        return v_local, yr, d.ok

    @guarded
    def _on_sp_vel(self, msg: Twist):
        self._touch("setpoint_velocity/cmd_vel_unstamped")
        r = self._velocity(msg.linear, msg.angular.z, "setpoint_velocity/cmd_vel_unstamped")
        if r is None:
            return
        v, yr, _ = r
        out = Twist()
        out.linear.x, out.linear.y, out.linear.z = float(v.x), float(v.y), float(v.z)
        out.angular.z = float(yr)
        self.p_sp_vel.publish(out)

    @guarded
    def _on_sp_vel_stamped(self, msg: TwistStamped):
        self._touch("setpoint_velocity/cmd_vel")
        r = self._velocity(msg.twist.linear, msg.twist.angular.z, "setpoint_velocity/cmd_vel")
        if r is None:
            return
        v, yr, _ = r
        out = TwistStamped()
        out.header.stamp = self._now_msg()
        out.header.frame_id = "map"
        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = float(v.x), float(v.y), float(v.z)
        out.twist.angular.z = float(yr)
        self.p_sp_vel_st.publish(out)

    @guarded
    def _on_raw_local(self, msg: PositionTarget):
        self._touch("setpoint_raw/local")
        frame, mask = int(msg.coordinate_frame), int(msg.type_mask) & 0x0FFF
        pos = Vec3(msg.position.x, msg.position.y, msg.position.z)
        vel = Vec3(msg.velocity.x, msg.velocity.y, msg.velocity.z)
        if not (mask & IGNORE_YAW) and not math.isfinite(msg.yaw):
            mask |= IGNORE_YAW
        pos_fence = pos
        if frame == FRAME_LOCAL_NED and self.is_global and (mask & POS_BITS) != POS_BITS:
            pos_fence = self._local_to_fence(pos)
            if pos_fence is None:
                self._error("setpoint_raw/local: no fresh local+GPS position to check it against the outdoor fence",
                            key="raw_local_nopos")
                return
        d = self.cg.check_raw_local(frame, mask, pos_fence, vel, msg.yaw_rate, self._snapshot())
        if not d.ok:
            self._reject(d, "setpoint_raw/local")
            if d.value is None:
                return  # position-type: dropping is safe (ArduPilot holds the last target)
        elif d.modified:
            self._error(f"setpoint_raw/local: {d.reason}", key=f"raw:{d.key}")
        val = dict(d.value)
        out_frame, out_mask = int(val["frame"]), int(val["type_mask"])
        body = out_frame in (FRAME_BODY_NED, FRAME_BODY_OFFSET_NED)
        p, v = (pos, val["vel"]) if d.ok else (Vec3(0, 0, 0), Vec3(0, 0, 0))
        yaw = msg.yaw if math.isfinite(msg.yaw) else 0.0
        if not body and not self.is_global and d.ok:
            p = self.a2l.point(pos) if out_frame == FRAME_LOCAL_NED else self.a2l.vector(pos)
            v = self.a2l.vector(v)
            if not (out_mask & IGNORE_YAW):
                yaw = self.a2l.heading(yaw)
        out = PositionTarget()
        out.header.stamp = self._now_msg()
        out.header.frame_id = "map"
        out.coordinate_frame = out_frame
        out.type_mask = out_mask
        out.position.x, out.position.y, out.position.z = float(p.x), float(p.y), float(p.z)
        out.velocity.x, out.velocity.y, out.velocity.z = float(v.x), float(v.y), float(v.z)
        out.yaw = float(yaw)
        out.yaw_rate = float(val["yaw_rate"])
        self.p_raw_local.publish(out)
        self._note_velocity(v if (out_mask & VEL_BITS) != VEL_BITS else Vec3(0, 0, 0))

    def _global_target(self, lat, lon, alt, what) -> bool:
        if not self.is_global:
            self._error(f"{what}: the active fence profile '{self.ds.profile_name}' is indoor/local; "
                        f"use local setpoints", key=f"{what}:indoor")
            return False
        if not (math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt)):
            self._error(f"{what}: NaN/inf in target", key=f"{what}:nan")
            return False
        e, n = self.ds.fence.to_plane(lat, lon)
        d = self.cg.check_position(Vec3(e, n, alt), self._snapshot())
        if not d.ok:
            hint = " (altitude is RELATIVE to home in this lab, not AMSL)" if "alt" in d.reason else ""
            self._error(f"{what}: {d.reason}{hint}", key=f"{what}:{d.key}")
            return False
        return True

    @guarded
    def _on_sp_global(self, msg: GeoPoseStamped):
        self._touch("setpoint_position/global")
        lat, lon, alt = msg.pose.position.latitude, msg.pose.position.longitude, msg.pose.position.altitude
        if not self._global_target(lat, lon, alt, "setpoint_position/global"):
            return
        out = GlobalPositionTarget()
        out.header.stamp = self._now_msg()
        out.coordinate_frame = FRAME_GLOBAL_REL_ALT_INT
        out.type_mask = VEL_BITS | ACC_BITS | IGNORE_YAW_RATE
        out.latitude, out.longitude, out.altitude = float(lat), float(lon), float(alt)
        q = msg.pose.orientation
        if _quat_ok(q):
            out.yaw = float(geo.yaw_from_quat(q.x, q.y, q.z, q.w))
        else:
            out.type_mask |= IGNORE_YAW
        self.p_raw_global.publish(out)
        self._note_velocity(Vec3(0, 0, 0))

    @guarded
    def _on_raw_global(self, msg: GlobalPositionTarget):
        self._touch("setpoint_raw/global")
        what = "setpoint_raw/global"
        if msg.coordinate_frame not in (FRAME_GLOBAL_REL_ALT, FRAME_GLOBAL_REL_ALT_INT):
            self._error(f"{what}: coordinate_frame must be 6 (FRAME_GLOBAL_REL_ALT) - altitude relative to home",
                        key=f"{what}:frame")
            return
        m = int(msg.type_mask) & 0x0FFF
        if (m & POS_BITS) != 0 or (m & VEL_BITS) != VEL_BITS or (m & ACC_BITS) != ACC_BITS or (m & FORCE):
            self._error(f"{what}: only full position targets are allowed (ignore velocity and acceleration)",
                        key=f"{what}:mask")
            return
        if not self._global_target(msg.latitude, msg.longitude, msg.altitude, what):
            return
        out = GlobalPositionTarget()
        out.header.stamp = self._now_msg()
        out.coordinate_frame = msg.coordinate_frame
        out.type_mask = m | IGNORE_YAW_RATE | (0 if math.isfinite(msg.yaw) else IGNORE_YAW)
        out.latitude, out.longitude, out.altitude = float(msg.latitude), float(msg.longitude), float(msg.altitude)
        out.yaw = float(msg.yaw) if math.isfinite(msg.yaw) else 0.0
        self.p_raw_global.publish(out)
        self._note_velocity(Vec3(0, 0, 0))

    # ----- services: student -> private -----
    def _call(self, client, req, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None, f"MAVROS service {client.srv_name} unavailable (is MAVROS connected?)"
        fut = client.call_async(req)
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None, f"{client.srv_name} timed out"
        try:
            return fut.result(), None
        except Exception as e:
            return None, str(e)

    def set_mode(self, mode: str, wait: float = 3.0) -> Tuple[bool, str]:
        req = SetMode.Request()
        req.custom_mode = mode
        res, err = self._call(self.c_mode, req)
        if err or not res.mode_sent:
            return False, err or f"FCU rejected mode {mode}"
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            with self.lock:
                if self.vs.mode == mode:
                    return True, f"mode {mode}"
            time.sleep(0.05)
        return False, f"mode {mode} sent but the drone still reports {self.vs.mode}"

    def arm(self, value: bool, wait: float = 3.0) -> Tuple[bool, str]:
        req = CommandBool.Request()
        req.value = value
        res, err = self._call(self.c_arm, req)
        if err or not res.success:
            return False, err or f"FCU refused to {'arm' if value else 'disarm'} (check pre-arm messages in " \
                                 f"{self.sns}/statustext/recv)"
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            with self.lock:
                if self.vs.armed == value:
                    return True, "armed" if value else "disarmed"
            time.sleep(0.05)
        return False, "arming command accepted but state did not change"

    def arm_sequence(self) -> Tuple[bool, str]:
        ok, msg = self.set_mode("LOITER")
        if not ok:
            return False, "LOITER: " + msg
        ok, msg = self.arm(True)
        if not ok:
            return False, "ARM: " + msg
        ok, msg = self.set_mode("GUIDED")
        if not ok:
            return False, "GUIDED: " + msg
        return True, "LOITER -> ARM -> GUIDED done; take off within ~10 s or ArduPilot disarms again"

    def _busy(self, res, what: str):
        self._error(f"{what}: too many service calls in progress, try again", key="svc_busy")
        if hasattr(res, "success"):
            res.success, res.result = False, 1
        else:
            res.mode_sent = False
        return res

    @guarded
    def _srv_arming(self, req, res):
        self._touch("cmd/arming")
        if not self._svc_sem.acquire(blocking=False):
            return self._busy(res, "cmd/arming")
        try:
            d = self.cg.check_arm(bool(req.value), self._snapshot())
            if not d.ok:
                self._reject(d, "cmd/arming")
                res.success, res.result = False, 1
                return res
            action = d.value
            if action in ("arm_sequence", "arm") and not self._seq_lock.acquire(blocking=False):
                self._error("cmd/arming: an arming sequence is already running", key="arm_busy")
                res.success, res.result = False, 1
                return res
            try:
                if action == "noop":
                    ok, msg = True, "already armed"
                elif action == "arm_sequence":
                    ok, msg = self.arm_sequence()
                    if ok:
                        self.cg.student_mode = "GUIDED"
                elif action == "arm":
                    ok, msg = self.arm(True)
                elif action == "land_instead":
                    ok, msg = self.set_mode("LAND")
                    self.cg.student_mode = "LAND"
                    msg = "in the air: LAND instead of disarm (" + msg + ")"
                else:
                    ok, msg = self.arm(False)
            finally:
                if action in ("arm_sequence", "arm"):
                    self._seq_lock.release()
            if not ok:
                self._error(f"cmd/arming: {msg}", key="arming_fail", force=True)
            else:
                self.log(f"[gate] cmd/arming({req.value}): {msg}")
            res.success, res.result = ok, 0 if ok else 1
            return res
        finally:
            self._svc_sem.release()

    @guarded
    def _srv_mode(self, req, res):
        self._touch("set_mode")
        if not self._svc_sem.acquire(blocking=False):
            return self._busy(res, "set_mode")
        try:
            d = self.cg.check_mode(req.custom_mode, self._snapshot())
            if not d.ok:
                self._reject(d, "set_mode")
                res.mode_sent = False
                return res
            r = SetMode.Request()
            r.custom_mode = d.value
            out, err = self._call(self.c_mode, r)
            res.mode_sent = bool(out and out.mode_sent)
            if res.mode_sent:
                self.cg.student_mode = d.value
            else:
                self._error(f"set_mode {d.value}: {err or 'FCU rejected the mode'}", key="mode_fail", force=True)
            return res
        finally:
            self._svc_sem.release()

    @guarded
    def _srv_takeoff(self, req, res):
        self._touch("cmd/takeoff")
        if not self._svc_sem.acquire(blocking=False):
            return self._busy(res, "cmd/takeoff")
        try:
            d = self.cg.check_takeoff(float(req.altitude), self._snapshot())
            if not d.ok:
                self._reject(d, "cmd/takeoff")
                res.success, res.result = False, 1
                return res
            out, err = self._call(self.c_takeoff, req)
            res.success = bool(out and out.success)
            res.result = out.result if out else 1
            if not res.success:
                self._error(f"cmd/takeoff: {err or 'FCU rejected takeoff'}", key="takeoff_fail", force=True)
            return res
        finally:
            self._svc_sem.release()

    @guarded
    def _srv_land(self, req, res):
        # own callback group, no semaphore: landing must never wait behind other calls
        self._touch("cmd/land")
        out, err = self._call(self.c_land, req)
        res.success = bool(out and out.success)
        res.result = out.result if out else 1
        if not res.success:
            # fall back to LAND mode, landing must never fail because of a parameter detail
            ok, msg = self.set_mode("LAND")
            res.success = ok
            if not ok:
                self._error(f"cmd/land: {err or 'rejected'}; LAND mode also failed: {msg}", key="land_fail",
                            force=True)
        if res.success:
            self.cg.student_mode = "LAND"
        return res

    # ----- instructor commands (admin socket) -----
    def admin(self, cmd: dict) -> dict:
        c = cmd.get("cmd")
        lock = bool(cmd.get("lock", True))   # the Demo app lands without locking
        if c in ("land", "brake", "mode"):
            mode = {"land": "LAND", "brake": "BRAKE"}.get(c) or str(cmd.get("mode", "")).upper()
            if lock and mode != "GUIDED":
                with self.lock:
                    self.cg.lock(f"instructor selected {mode}")
            ok, msg = self.set_mode(mode)
            if ok and lock:
                msg += " (student commands locked until RELEASE LOCK)"
        elif c == "release_lock":
            with self.lock:
                self.cg.release()
            ok, msg = True, "lock released"
            self._error("instructor released the lock", key="release", force=True)
        elif c == "disarm":
            if self._snapshot().in_air:
                return {"ok": False, "msg": "refusing to disarm in the air (use LAND, or KILL to cut motors)"}
            ok, msg = self.arm(False)
        elif c == "kill":
            req = CommandLong.Request()
            req.command = 400        # MAV_CMD_COMPONENT_ARM_DISARM
            req.param1 = 0.0
            req.param2 = 21196.0     # force, even in flight
            out, err = self._call(self.c_cmd, req)
            ok = bool(out and out.success)
            msg = "motors killed" if ok else (err or "kill rejected")
        elif c == "arm_takeoff":
            ok, msg = self.arm_and_takeoff(float(cmd.get("alt", 1.0)))
        else:
            return {"ok": False, "msg": f"unknown command {c}"}
        return {"ok": ok, "msg": msg}

    def arm_and_takeoff(self, alt: float) -> Tuple[bool, str]:
        if not self._seq_lock.acquire(blocking=False):
            return False, "an arming sequence is already running"
        try:
            st = self._snapshot()
            if self.cg.locked:
                return False, "locked (fence breach or instructor) - release the lock first"
            if not st.armed:
                if self.ds.rules.arm_sequence == "loiter_arm_guided":
                    ok, msg = self.arm_sequence()
                else:
                    ok, msg = self.set_mode("GUIDED")
                    if ok:
                        ok, msg = self.arm(True)
                if not ok:
                    return False, msg
            elif st.mode != "GUIDED":
                ok, msg = self.set_mode("GUIDED")
                if not ok:
                    return False, msg
            d = self.cg.check_takeoff(alt, self._snapshot())
            if not d.ok:
                return False, d.reason
            req = CommandTOL.Request()
            req.altitude = float(alt)
            out, err = self._call(self.c_takeoff, req)
            if not (out and out.success):
                return False, err or "takeoff rejected"
            self.cg.student_mode = "GUIDED"
            return True, f"taking off to {alt:.1f} m"
        finally:
            self._seq_lock.release()

    # =================================================================== status
    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            v = self.vs
            pos = [round(v.pos.x, 3), round(v.pos.y, 3), round(v.pos.z, 3)] if v.pos else None
            st = {
                "sysid": self.ds.sysid, "student_ns": self.sns, "student_domain": self.ds.student_domain,
                "private_domain": self.ds.private_domain, "profile": self.ds.profile_name,
                "profile_label": self.ds.profile_label, "verified": self.ds.profile_verified,
                "fence": self.ds.fence.describe(), "connected": v.connected, "armed": v.armed, "mode": v.mode,
                "landed_state": v.landed_state, "in_air": v.in_air if v.connected else False,
                "pos": pos, "yaw": round(v.yaw, 3) if v.yaw is not None else None,
                "pose_age": round(now - v.pos_stamp, 2) if v.pos else None,
                "pose_rate": sum(1 for t in self.pose_times if now - t < 1.0),
                "gps": [self.global_fix[0], self.global_fix[1]] if self.global_fix else None,
                "battery": dict(self.battery), "locked": self.cg.locked, "lock_reason": self.cg.lock_reason,
                "stats": dict(self.cg.stats),
                "student_activity": {k: round(now - t, 1) for k, t in self.student_activity.items()},
                "relayed_topics": sum(1 for r in list(self.relays.values()) if r),
                "last_breach": self.last_breach, "uptime": round(now - self.started, 1),
                "limits": {"max_speed_xy": self.ds.limits.max_speed_xy, "max_speed_z": self.ds.limits.max_speed_z},
                "allowed_modes": list(self.ds.rules.allowed_modes),
                "callback_errors": self.callback_errors, "executors_alive": self.executors_alive,
            }
            st["errors"] = list(self.errors)[-12:]
        return st

    @guarded
    def _publish_status(self):
        st = self.status()
        st.pop("errors", None)
        self.p_status.publish(String(data=json.dumps(st)))

    @guarded
    def _publish_fence(self):
        f = self.ds.fence.describe()
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self._now_msg()
        m.ns = "dronelab_fence"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.1, 0.9
        m.pose.orientation.w = 1.0
        pts = []
        if f["type"] == "box":
            (x0, x1), (y0, y1), (z0, z1) = f["x"], f["y"], (0.0, f["z"][1])
            c = [(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
            for i in range(4):
                a, b = c[i], c[(i + 1) % 4]
                for z in (z0, z1):
                    pts += [(a[0], a[1], z), (b[0], b[1], z)]
                pts += [(a[0], a[1], z0), (a[0], a[1], z1)]
        else:
            poly = f.get("plane_points")
            if poly is None or self.ds.fence.kind == "global":
                # draw global fences in the local frame once the local<->GPS offset is known
                with self.lock:
                    off = self.plane_offset
                if off is None:
                    return
                if f["type"] == "circle":
                    poly = [(f["radius"] * math.cos(a * math.pi / 18), f["radius"] * math.sin(a * math.pi / 18))
                            for a in range(36)]
                poly = [(x - off.x, y - off.y) for x, y in poly]
                z0, z1 = f["z"][0] - off.z, f["z"][1] - off.z
            else:
                z0, z1 = f["z"]
            for i in range(len(poly)):
                a, b = poly[i], poly[(i + 1) % len(poly)]
                for z in (z0, z1):
                    pts += [(a[0], a[1], z), (b[0], b[1], z)]
        for x, y, z in pts:
            m.points.append(Point(x=float(x), y=float(y), z=float(z)))
        self.p_fence.publish(m)
