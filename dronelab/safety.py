"""Command validation for the student gate. Pure Python, no ROS.

Every student command goes through `CommandGate`. A command is either
forwarded unchanged, forwarded modified (velocity limited near the fence),
or rejected with a human-readable reason that is published on the
student error topic.

Fence coordinates
  * local fences (indoor): arena frame, ENU metres. z is height above the arena floor.
  * global fences (outdoor): east/north metres in the fence's tangent plane,
    z is altitude relative to home (where the drone armed).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from . import geo
from .geo import Vec2, Vec3

# MAVLink POSITION_TARGET_TYPEMASK bits (mavros_msgs/PositionTarget.IGNORE_*)
IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
FORCE = 512
IGNORE_YAW, IGNORE_YAW_RATE = 1024, 2048
POS_BITS = IGNORE_PX | IGNORE_PY | IGNORE_PZ
VEL_BITS = IGNORE_VX | IGNORE_VY | IGNORE_VZ
ACC_BITS = IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ

FRAME_LOCAL_NED = 1
FRAME_LOCAL_OFFSET_NED = 7
FRAME_BODY_NED = 8
FRAME_BODY_OFFSET_NED = 9
FRAME_GLOBAL_INT = 5
FRAME_GLOBAL_REL_ALT = 3
FRAME_GLOBAL_REL_ALT_INT = 6
FRAME_GLOBAL_TERRAIN_ALT_INT = 11

LANDED_UNDEFINED, LANDED_ON_GROUND, LANDED_IN_AIR, LANDED_TAKEOFF, LANDED_LANDING = 0, 1, 2, 3, 4

# ROS 2 services do not work between Fast DDS (the Humble default) and CycloneDDS (the Deck): the request
# arrives shifted (empty strings, zeros, false) and the reply never comes back. Topics work.
RMW_HINT = ("If you did send a value, your PC uses Fast DDS: ROS 2 services do not work between Fast DDS "
            "and the Deck's CycloneDDS (topics do). On the student PC: "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (apt install ros-humble-rmw-cyclonedds-cpp)")

# zero-velocity hover in the local frame (velocity + yaw-rate used, everything else ignored)
HOVER_RAW = dict(frame=1, type_mask=1 | 2 | 4 | 64 | 128 | 256 | 1024, pos=Vec3(0, 0, 0), vel=Vec3(0, 0, 0),
                 yaw_rate=0.0)


# --------------------------------------------------------------------------
# Fences
# --------------------------------------------------------------------------
class Fence:
    """Base class. Coordinates are 'fence coordinates' (see module docstring)."""

    kind = "local"  # or "global"
    margin = 0.3
    zmin = 0.0
    zmax = 2.5
    # Velocity limiting aims this far INSIDE the fence: ArduPilot's velocity response lags the
    # command, so a drone slowed exactly to the line still crosses it a little (measured in SITL).
    stop_buffer = 0.2

    def _stop(self, distance: float, decel: float, lookahead: float) -> float:
        return geo.simple_stop_speed(distance - self.stop_buffer, decel, lookahead)

    def check_point(self, p: Vec3) -> Optional[str]:
        raise NotImplementedError

    def breached(self, p: Vec3) -> Optional[str]:
        raise NotImplementedError

    def check_path(self, a: Vec3, b: Vec3) -> Optional[str]:
        return None

    def limit_velocity(self, p: Vec3, v: Vec3, lookahead: float, decel: float) -> Tuple[Vec3, Optional[str]]:
        raise NotImplementedError

    def describe(self) -> dict:
        raise NotImplementedError

    # global fences convert lat/lon to plane coordinates
    plane: Optional[geo.LocalTangent] = None

    def to_plane(self, lat: float, lon: float) -> Vec2:
        if self.plane is None:
            raise ValueError("local fence has no geodetic reference")
        return self.plane.to_en(lat, lon)

    def _z_limit_velocity(self, p: Vec3, vz: float, lookahead: float, decel: float) -> Tuple[float, Optional[str]]:
        if vz > 0:
            allowed = self._stop(self.zmax - p.z, decel, lookahead)
            if vz > allowed:
                return allowed, "ceiling"
        elif vz < 0:
            allowed = geo.simple_stop_speed(p.z - self.zmin, decel, lookahead)
            if -vz > allowed:
                return -allowed, "floor (use the land service to land)"
        return vz, None


@dataclass
class BoxFence(Fence):
    xmin: float = -0.5
    xmax: float = 4.0
    ymin: float = -0.5
    ymax: float = 6.0
    zmin: float = 0.5
    zmax: float = 2.5
    margin: float = 0.3
    kind = "local"

    def __post_init__(self):
        if not (self.xmin < self.xmax and self.ymin < self.ymax and self.zmin < self.zmax):
            raise ValueError("box fence: every min must be smaller than its max")
        if self.margin < 0:
            raise ValueError("box fence: margin must be >= 0")

    def check_point(self, p: Vec3) -> Optional[str]:
        bad = []
        if not self.xmin <= p.x <= self.xmax:
            bad.append(f"x={p.x:.2f} not in [{self.xmin}, {self.xmax}]")
        if not self.ymin <= p.y <= self.ymax:
            bad.append(f"y={p.y:.2f} not in [{self.ymin}, {self.ymax}]")
        if not self.zmin <= p.z <= self.zmax:
            bad.append(f"z={p.z:.2f} not in [{self.zmin}, {self.zmax}]")
        return "; ".join(bad) or None

    def breached(self, p: Vec3) -> Optional[str]:
        m = self.margin
        bad = []
        if not (self.xmin - m <= p.x <= self.xmax + m):
            bad.append(f"x={p.x:.2f}")
        if not (self.ymin - m <= p.y <= self.ymax + m):
            bad.append(f"y={p.y:.2f}")
        if p.z > self.zmax + m:
            bad.append(f"z={p.z:.2f}")
        return ("outside fence: " + ", ".join(bad)) if bad else None

    def limit_velocity(self, p, v, lookahead, decel):
        out = [v.x, v.y]
        why = []
        for i, (pos, lo, hi, name) in enumerate(((p.x, self.xmin, self.xmax, "x"), (p.y, self.ymin, self.ymax, "y"))):
            vi = out[i]
            if vi > 0:
                allowed = self._stop(hi - pos, decel, lookahead)
                if vi > allowed:
                    out[i] = allowed
                    why.append(f"+{name} wall")
            elif vi < 0:
                allowed = self._stop(pos - lo, decel, lookahead)
                if -vi > allowed:
                    out[i] = -allowed
                    why.append(f"-{name} wall")
        vz, zwhy = self._z_limit_velocity(p, v.z, lookahead, decel)
        if zwhy:
            why.append(zwhy)
        return Vec3(out[0], out[1], vz), (", ".join(why) or None)

    def describe(self):
        return {"type": "box", "frame": "local", "x": [self.xmin, self.xmax], "y": [self.ymin, self.ymax],
                "z": [self.zmin, self.zmax], "margin": self.margin}


@dataclass
class PolygonFence(Fence):
    """Polygon prism. points are (x, y) metres (local) or (lat, lon) degrees (global)."""

    points: Sequence[Tuple[float, float]] = ()
    zmin: float = 2.0
    zmax: float = 30.0
    margin: float = 2.0
    is_global: bool = True

    def __post_init__(self):
        pts = [(float(a), float(b)) for a, b in self.points]
        if self.is_global:
            lat0 = sum(p[0] for p in pts) / len(pts)
            lon0 = sum(p[1] for p in pts) / len(pts)
            self.plane = geo.LocalTangent(lat0, lon0)
            self.kind = "global"
            self.poly = [self.plane.to_en(lat, lon) for lat, lon in pts]
        else:
            self.plane = None
            self.kind = "local"
            self.poly = pts
        if not geo.offset_polygon_inward_ok(self.poly):
            raise ValueError("polygon fence needs >= 3 vertices and a non-zero area")
        if geo.polygon_self_intersects(self.poly):
            raise ValueError("polygon fence edges intersect each other")
        if not self.zmin < self.zmax:
            raise ValueError("polygon fence: z/alt min must be smaller than max")

    def check_point(self, p):
        bad = []
        if not geo.point_in_polygon((p.x, p.y), self.poly):
            bad.append("outside polygon")
        if not self.zmin <= p.z <= self.zmax:
            bad.append(f"alt={p.z:.1f} not in [{self.zmin}, {self.zmax}]")
        return "; ".join(bad) or None

    def breached(self, p):
        bad = []
        xy = (p.x, p.y)
        if not geo.point_in_polygon(xy, self.poly) and geo.dist_to_polygon_edge(xy, self.poly) > self.margin:
            bad.append(f"{geo.dist_to_polygon_edge(xy, self.poly):.1f} m outside polygon")
        if p.z > self.zmax + self.margin:
            bad.append(f"alt={p.z:.1f}")
        return ("outside fence: " + ", ".join(bad)) if bad else None

    def check_path(self, a, b):
        if geo.segment_leaves_polygon((a.x, a.y), (b.x, b.y), self.poly):
            return "straight path to the target leaves the fence"
        return None

    def limit_velocity(self, p, v, lookahead, decel):
        why = []
        vxy = v.norm_xy()
        vx, vy = v.x, v.y
        if vxy > 1e-6:
            d = (v.x / vxy, v.y / vxy)
            inside = geo.point_in_polygon((p.x, p.y), self.poly)
            if inside:
                dist = geo.ray_distance_to_polygon((p.x, p.y), d, self.poly)
                allowed = self._stop(dist, decel, lookahead)
            else:
                # outside: only allow moving back toward the inside
                ahead = (p.x + d[0] * 0.5, p.y + d[1] * 0.5)
                closer = geo.dist_to_polygon_edge(ahead, self.poly) < geo.dist_to_polygon_edge((p.x, p.y), self.poly)
                allowed = vxy if closer or geo.point_in_polygon(ahead, self.poly) else 0.0
            if vxy > allowed:
                k = allowed / vxy
                vx, vy = v.x * k, v.y * k
                why.append("fence edge")
        vz, zwhy = self._z_limit_velocity(p, v.z, lookahead, decel)
        if zwhy:
            why.append(zwhy)
        return Vec3(vx, vy, vz), (", ".join(why) or None)

    def describe(self):
        return {"type": "polygon", "frame": self.kind, "points": [list(p) for p in self.points],
                "plane_points": [list(p) for p in self.poly], "z": [self.zmin, self.zmax], "margin": self.margin}


@dataclass
class CircleFence(Fence):
    lat: float = 0.0
    lon: float = 0.0
    radius: float = 50.0
    zmin: float = 2.0
    zmax: float = 30.0
    margin: float = 2.0
    kind = "global"

    def __post_init__(self):
        if self.radius <= 0:
            raise ValueError("circle fence: radius must be > 0")
        if not self.zmin < self.zmax:
            raise ValueError("circle fence: alt min must be smaller than max")
        self.plane = geo.LocalTangent(self.lat, self.lon)

    def check_point(self, p):
        bad = []
        r = math.hypot(p.x, p.y)
        if r > self.radius:
            bad.append(f"{r:.1f} m from centre > radius {self.radius}")
        if not self.zmin <= p.z <= self.zmax:
            bad.append(f"alt={p.z:.1f} not in [{self.zmin}, {self.zmax}]")
        return "; ".join(bad) or None

    def breached(self, p):
        bad = []
        r = math.hypot(p.x, p.y)
        if r > self.radius + self.margin:
            bad.append(f"{r:.1f} m from centre")
        if p.z > self.zmax + self.margin:
            bad.append(f"alt={p.z:.1f}")
        return ("outside fence: " + ", ".join(bad)) if bad else None

    def limit_velocity(self, p, v, lookahead, decel):
        why = []
        vxy = v.norm_xy()
        vx, vy = v.x, v.y
        if vxy > 1e-6:
            d = (v.x / vxy, v.y / vxy)
            if math.hypot(p.x, p.y) <= self.radius:
                dist = geo.ray_distance_to_circle((p.x, p.y), d, self.radius)
                allowed = self._stop(dist, decel, lookahead)
            else:
                inward = (p.x * d[0] + p.y * d[1]) < 0
                allowed = vxy if inward else 0.0
            if vxy > allowed:
                k = allowed / vxy
                vx, vy = v.x * k, v.y * k
                why.append("fence edge")
        vz, zwhy = self._z_limit_velocity(p, v.z, lookahead, decel)
        if zwhy:
            why.append(zwhy)
        return Vec3(vx, vy, vz), (", ".join(why) or None)

    def describe(self):
        return {"type": "circle", "frame": "global", "center": [self.lat, self.lon], "radius": self.radius,
                "z": [self.zmin, self.zmax], "margin": self.margin}


def fence_from_config(cfg: dict) -> Fence:
    t = str(cfg.get("type", "box")).lower()
    margin = float(cfg.get("margin_m", 0.3 if t == "box" else 2.0))
    if t == "box":
        x, y, z = cfg.get("x", [-0.5, 4.0]), cfg.get("y", [-0.5, 6.0]), cfg.get("z", [0.5, 2.5])
        return BoxFence(float(x[0]), float(x[1]), float(y[0]), float(y[1]), float(z[0]), float(z[1]), margin)
    if t == "polygon":
        frame = str(cfg.get("frame", "global")).lower()
        alt = cfg.get("alt_rel_m", cfg.get("z", [2.0, 30.0]))
        return PolygonFence(points=[tuple(p) for p in cfg["points"]], zmin=float(alt[0]), zmax=float(alt[1]),
                            margin=margin, is_global=(frame == "global"))
    if t == "circle":
        alt = cfg.get("alt_rel_m", [2.0, 30.0])
        c = cfg["center"]
        return CircleFence(lat=float(c[0]), lon=float(c[1]), radius=float(cfg["radius_m"]),
                           zmin=float(alt[0]), zmax=float(alt[1]), margin=margin)
    raise ValueError(f"unknown fence type '{t}' (use box, polygon or circle)")


# --------------------------------------------------------------------------
# Vehicle state as seen by the gate
# --------------------------------------------------------------------------
@dataclass
class VehicleState:
    pos: Optional[Vec3] = None      # fence coordinates
    yaw: Optional[float] = None     # ENU yaw in fence frame (rad)
    pos_stamp: float = 0.0          # time.monotonic() of last pos update
    armed: bool = False
    mode: str = ""
    connected: bool = False
    landed_state: int = LANDED_UNDEFINED

    def pos_fresh(self, max_age: float, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return self.pos is not None and (now - self.pos_stamp) <= max_age

    @property
    def in_air(self) -> bool:
        if not self.armed:
            return False
        if self.landed_state in (LANDED_IN_AIR, LANDED_TAKEOFF, LANDED_LANDING):
            return True
        if self.landed_state == LANDED_ON_GROUND:
            return False
        # EXTENDED_SYS_STATE not received (ArduCopter only sends it on request):
        # armed and clearly above the ground counts as flying; no height known -> assume flying
        if self.pos is not None:
            return self.pos.z > 0.25
        return True


@dataclass
class Limits:
    max_speed_xy: float = 1.0
    max_speed_z: float = 0.5
    max_yaw_rate: float = math.radians(45)
    lookahead_s: float = 1.0
    decel: float = 1.5
    pose_timeout_s: float = 0.5
    stop_buffer_m: float = 0.2


@dataclass
class Rules:
    arm_sequence: str = "loiter_arm_guided"   # or "direct"
    guided_requires_armed: bool = True
    # RTL climbs to RTL_ALT and flies at WPNAV speed (ignores the gate) -> not allowed indoors by default;
    # ALT_HOLD/POSHOLD need RC sticks, students have none.
    allowed_modes: List[str] = field(default_factory=lambda: ["GUIDED", "LAND", "BRAKE"])
    breach_action: str = "land"               # land | brake | none
    breach_samples: int = 3                   # consecutive out-of-fence samples before acting
    # local (indoor) fences: a drone on the ground must read z = 0 +- this. Otherwise its height reference
    # drifted (barometer) or the arena frame's floor is not z = 0, and the fence floor/ceiling, take-off
    # heights and "in the air" detection are all off by that much -> students cannot arm. 0 = no check.
    ground_tolerance_m: float = 0.5

    def __post_init__(self):
        self.allowed_modes = [m.upper() for m in self.allowed_modes]
        if "LAND" not in self.allowed_modes:
            self.allowed_modes.append("LAND")     # landing must always be possible


@dataclass
class Decision:
    ok: bool
    reason: str = ""
    value: object = None       # possibly modified command
    modified: bool = False
    key: str = ""              # stable id for rate-limiting identical errors

    @staticmethod
    def reject(reason: str, key: str = "") -> "Decision":
        return Decision(False, reason, key=key or reason[:40])


# --------------------------------------------------------------------------
# The gate logic
# --------------------------------------------------------------------------
class CommandGate:
    def __init__(self, fence: Fence, limits: Limits, rules: Rules):
        fence.stop_buffer = limits.stop_buffer_m
        self.fence = fence
        self.limits = limits
        self.rules = rules
        self.locked = False
        self.lock_reason = ""
        self._breach_count = 0
        self._rearm_needed = False  # after a release: monitor again only once back inside
        self.student_mode = ""      # last mode the STUDENT selected (see check_mode)
        self.stats = {"accepted": 0, "modified": 0, "rejected": 0}

    # ---------------- helpers -----------------
    def _count(self, d: Decision) -> Decision:
        if not d.ok:
            self.stats["rejected"] += 1
        elif d.modified:
            self.stats["modified"] += 1
        else:
            self.stats["accepted"] += 1
        return d

    def _locked(self) -> Optional[Decision]:
        if self.locked:
            return Decision.reject(f"LOCKED after fence breach ({self.lock_reason}); only land is allowed "
                                   f"until the instructor releases the lock", key="locked")
        return None

    def pilot_control(self, state: VehicleState) -> Optional[str]:
        """Name of the mode if the RC pilot (or an ArduPilot failsafe) has taken over, else None.

        "Taken over" = armed, not in GUIDED, and in a mode that neither the students nor the gate
        selected. Students then cannot command anything (setpoints, modes, landing) until the drone
        is back in GUIDED, e.g. the pilot hands control back with the mode switch.
        """
        if not state.armed or self.locked:
            return None
        cur = (state.mode or "").upper()
        if not cur or cur == "GUIDED" or cur == self.student_mode:
            return None
        return cur

    def ground_problem(self, state: VehicleState) -> Optional[str]:
        """Why the drone's height cannot be trusted (local fences, disarmed, on the ground), else None."""
        tol = self.rules.ground_tolerance_m
        if (tol <= 0 or self.fence.kind != "local" or state.armed or state.pos is None
                or state.landed_state in (LANDED_IN_AIR, LANDED_TAKEOFF, LANDED_LANDING)):
            return None
        z = state.pos.z
        if abs(z) <= tol:
            return None
        return (f"the drone stands on the ground but its height reads {z:+.2f} m (allowed +-{tol:g} m): the height "
                f"reference drifted (barometer? use OptiTrack height, EK3_SRC1_POSZ=6) or the arena floor is not "
                f"z=0 (arena_to_local z). Fence floor/ceiling and take-off heights would be off by {abs(z):.1f} m; "
                f"arming is blocked until it reads ~0 (reboot the flight controller or fix the source)")

    def _pilot(self, state: VehicleState) -> Optional[Decision]:
        m = self.pilot_control(state)
        if m:
            return Decision.reject(f"the RC pilot / a failsafe switched the drone to {m}: student commands are "
                                   f"ignored until it is back in GUIDED", key="pilot")
        return None

    def lock(self, reason: str):
        self.locked = True
        self.lock_reason = reason

    def release(self):
        """Instructor releases the lock. The breach monitor stays quiet until the drone has been back
        inside the fence, so the drone can be flown back in instead of being forced down again."""
        self.locked = False
        self.lock_reason = ""
        self._breach_count = 0
        self._rearm_needed = True

    def _clamp_speed(self, v: Vec3) -> Tuple[Vec3, bool]:
        L = self.limits
        changed = False
        vxy = v.norm_xy()
        x, y, z = v.x, v.y, v.z
        if vxy > L.max_speed_xy:
            k = L.max_speed_xy / vxy
            x, y = x * k, y * k
            changed = True
        if abs(z) > L.max_speed_z:
            z = math.copysign(L.max_speed_z, z)
            changed = True
        return Vec3(x, y, z), changed

    def clamp_yaw_rate(self, r: float) -> Tuple[float, bool]:
        if not math.isfinite(r):
            return 0.0, True
        m = self.limits.max_yaw_rate
        if abs(r) > m:
            return math.copysign(m, r), True
        return r, False

    # ---------------- position setpoints -----------------
    def check_position(self, target: Vec3, state: VehicleState, now: Optional[float] = None) -> Decision:
        """target in fence coordinates (arena frame or plane EN + rel alt)."""
        return self._count(self._check_position(target, state, now))

    def _check_position(self, target: Vec3, state: VehicleState, now: Optional[float]) -> Decision:
        if (d := self._locked() or self._pilot(state)):
            return d
        if not target.finite():
            return Decision.reject("setpoint contains NaN/inf", key="nan")
        why = self.fence.check_point(target)
        if why:
            return Decision.reject(f"setpoint outside fence: {why}", key="pos_out")
        if isinstance(self.fence, PolygonFence):
            if not state.pos_fresh(self.limits.pose_timeout_s, now):
                return Decision.reject("no fresh position yet, cannot check the path to the target", key="nopos")
            if self.fence.check_point(state.pos) is None:  # only meaningful from inside
                why = self.fence.check_path(state.pos, target)
                if why:
                    return Decision.reject(why, key="path")
        return Decision(True, value=target)

    # ---------------- velocity setpoints -----------------
    def check_velocity(self, v_world: Vec3, yaw_rate: float, state: VehicleState,
                       now: Optional[float] = None) -> Decision:
        """v_world in fence frame axes (ENU). Returns Decision.value = (Vec3, yaw_rate).

        A rejected velocity still carries value=(zero, 0): ArduPilot keeps executing the last
        velocity for a few seconds, so the caller must send a hover command instead of dropping.
        """
        return self._count(self._check_velocity(v_world, yaw_rate, state, now))

    def _check_velocity(self, v_world: Vec3, yaw_rate: float, state: VehicleState,
                        now: Optional[float]) -> Decision:
        hover = (Vec3(0, 0, 0), 0.0)
        if (d := self._locked()):
            d.value = hover
            return d
        if (d := self._pilot(state)):
            return d
        if not v_world.finite() or not math.isfinite(yaw_rate):
            return Decision(False, "velocity contains NaN/inf: replaced by hover", value=hover, key="nan")
        if not state.pos_fresh(self.limits.pose_timeout_s, now):
            return Decision(False, "no fresh position: velocity replaced by hover", value=hover, key="nopos_vel")
        v, spd = self._clamp_speed(v_world)
        v, why = self.fence.limit_velocity(state.pos, v, self.limits.lookahead_s, self.limits.decel)
        yr, yr_changed = self.clamp_yaw_rate(yaw_rate)
        reasons = []
        if spd:
            reasons.append("speed limit")
        if why:
            reasons.append("slowed at " + why)
        if yr_changed:
            reasons.append("yaw rate limit")
        modified = bool(reasons)
        return Decision(True, "; ".join(reasons), value=(v, yr), modified=modified,
                        key=("vel:" + (why or "spd")) if modified else "")

    # ---------------- raw PositionTarget (local) -----------------
    def check_raw_local(self, frame: int, type_mask: int, pos: Vec3, vel: Vec3, yaw_rate: float,
                        state: VehicleState, now: Optional[float] = None) -> Decision:
        """pos/vel already expressed in fence-frame axes for frames 1/7, body FLU for 8/9.

        Returns Decision.value = dict(frame, type_mask, pos, vel, yaw_rate) in the same conventions
        (velocities stay in the frame they came in, positions stay as given).
        """
        type_mask &= 0x0FFF   # only the defined bits
        wants_motion = (type_mask & VEL_BITS) != VEL_BITS or (type_mask & ACC_BITS) != ACC_BITS

        def rejected(reason: str, key: str) -> Decision:
            # Anything that looks like a velocity/acceleration command is answered with a hover:
            # ArduPilot would otherwise keep flying the previous velocity for GUID_TIMEOUT seconds.
            d = Decision.reject(reason, key)
            if wants_motion:
                d.value = HOVER_RAW
            return self._count(d)

        if self.locked:
            return rejected(self._locked().reason, "locked")
        if (d := self._pilot(state)):
            return self._count(d)
        if frame not in (FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED):
            return rejected(f"coordinate_frame {frame} not allowed (use 1, 7, 8 or 9)", "frame")
        if (type_mask & ACC_BITS) != ACC_BITS or (type_mask & FORCE):
            return rejected("acceleration/force setpoints are disabled in the lab", "accel")
        use_pos = (type_mask & POS_BITS) != POS_BITS
        use_vel = (type_mask & VEL_BITS) != VEL_BITS
        if use_pos and (type_mask & POS_BITS) != 0:
            return rejected("partial position setpoints (some axes ignored) are not allowed", "partial")
        if use_vel and (type_mask & VEL_BITS) != 0:
            return rejected("partial velocity setpoints are not allowed", "partial")
        if use_pos and use_vel:
            # ArduPilot integrates a feed-forward velocity into the position target for up to
            # GUID_TIMEOUT seconds, which can carry the target through the fence: drop it.
            type_mask |= VEL_BITS
            use_vel = False
            ff_dropped = True
        else:
            ff_dropped = False
        if not use_pos and not use_vel:
            # yaw / yaw-rate only: harmless, but clamp yaw rate
            yr, ch = self.clamp_yaw_rate(yaw_rate)
            return self._count(Decision(True, "yaw rate limit" if ch else "",
                                        value=dict(frame=frame, type_mask=type_mask, pos=pos, vel=vel, yaw_rate=yr),
                                        modified=ch))
        body = frame in (FRAME_BODY_NED, FRAME_BODY_OFFSET_NED)
        offset = frame != FRAME_LOCAL_NED
        out_vel = vel
        reasons = ["feed-forward velocity ignored (position target only)"] if ff_dropped else []
        if use_pos:
            if offset:
                if not state.pos_fresh(self.limits.pose_timeout_s, now) or state.yaw is None:
                    return self._count(Decision.reject("offset/body position needs a fresh position fix", key="nopos"))
                dx, dy = geo.rotate_xy(pos.x, pos.y, state.yaw) if body else (pos.x, pos.y)
                target = Vec3(state.pos.x + dx, state.pos.y + dy, state.pos.z + pos.z)
            else:
                target = pos
            dpos = self._check_position(target, state, now)
            if not dpos.ok:
                return self._count(dpos)
            out_vel = Vec3(0, 0, 0)
        else:  # velocity only
            if body:
                if state.yaw is None:
                    return self._count(Decision(False, "no heading yet: velocity replaced by hover",
                                                value=dict(frame=frame, type_mask=type_mask, pos=pos,
                                                           vel=Vec3(0, 0, 0), yaw_rate=0.0), key="nopos_vel"))
                wx, wy = geo.rotate_xy(vel.x, vel.y, state.yaw)
                vw = Vec3(wx, wy, vel.z)
            else:
                vw = vel
            dv = self._check_velocity(vw, yaw_rate, state, now)
            vnew, yr = dv.value if dv.value is not None else (Vec3(0, 0, 0), 0.0)
            if body:
                bx, by = geo.rotate_xy(vnew.x, vnew.y, -state.yaw)
                vnew = Vec3(bx, by, vnew.z)
            value = dict(frame=frame, type_mask=type_mask, pos=pos, vel=vnew, yaw_rate=yr)
            if not dv.ok:
                return self._count(Decision(False, dv.reason, value=value, key=dv.key))
            return self._count(Decision(True, dv.reason, value=value, modified=dv.modified, key=dv.key))
        yr, ch = self.clamp_yaw_rate(yaw_rate)
        if ch:
            reasons.append("yaw rate limit")
        return self._count(Decision(True, "; ".join(reasons),
                                    value=dict(frame=frame, type_mask=type_mask, pos=pos, vel=out_vel, yaw_rate=yr),
                                    modified=bool(reasons), key="ff" if ff_dropped else ""))

    # ---------------- services -----------------
    def check_takeoff(self, altitude: float, state: VehicleState) -> Decision:
        if (d := self._locked() or self._pilot(state)):
            return self._count(d)
        if not math.isfinite(altitude):
            return self._count(Decision.reject("takeoff altitude is NaN/inf"))
        if altitude == 0.0:
            return self._count(Decision.reject("takeoff altitude is 0. " + RMW_HINT, key="to_zero"))
        if not state.armed:
            return self._count(Decision.reject("takeoff rejected: not armed (call cmd/arming first)", key="to_arm"))
        if state.mode != "GUIDED":
            return self._count(Decision.reject(f"takeoff needs GUIDED mode (now {state.mode or 'unknown'})",
                                               key="to_mode"))
        if not self.fence.zmin <= altitude <= self.fence.zmax:
            return self._count(Decision.reject(
                f"takeoff altitude {altitude:.2f} m not in [{self.fence.zmin}, {self.fence.zmax}]", key="to_alt"))
        return self._count(Decision(True, value=altitude))

    def check_mode(self, mode: str, state: VehicleState) -> Decision:
        mode = (mode or "").upper().strip()
        if self.locked and mode != "LAND":
            return self._count(self._locked())
        if (d := self._pilot(state)):
            # the transmitter or an ArduPilot failsafe chose this mode - students must not take it back
            return self._count(d)
        if not mode:
            return self._count(Decision.reject("custom_mode is empty. " + RMW_HINT, key="mode_empty"))
        if mode not in self.rules.allowed_modes:
            return self._count(Decision.reject(f"mode {mode or '<empty>'} not allowed; allowed: "
                                               f"{', '.join(self.rules.allowed_modes)}", key="mode"))
        if mode == "GUIDED" and self.rules.guided_requires_armed and not state.armed:
            return self._count(Decision.reject("GUIDED is only allowed after arming (arm in LOITER first; "
                                               "cmd/arming does the whole sequence)", key="guided_arm"))
        return self._count(Decision(True, value=mode))

    def check_arm(self, value: bool, state: VehicleState) -> Decision:
        """Returns value = 'arm_sequence' | 'arm' | 'disarm' | 'land_instead' | 'noop'."""
        if (d := self._pilot(state)):
            return self._count(d)
        if value:
            if (d := self._locked()):
                return self._count(d)
            if state.armed:
                return self._count(Decision(True, "already armed", value="noop"))
            if (why := self.ground_problem(state)):
                return self._count(Decision.reject("arming refused: " + why, key="ground_z"))
            return self._count(Decision(True, value="arm_sequence" if self.rules.arm_sequence == "loiter_arm_guided"
                                        else "arm"))
        if state.armed and state.in_air:
            return self._count(Decision(True, "in the air: switching to LAND instead of cutting the motors",
                                        value="land_instead", modified=True))
        if not state.armed:
            return self._count(Decision(True, "already disarmed. " + RMW_HINT, value="noop_disarmed"))
        return self._count(Decision(True, value="disarm"))

    def check_land(self, state: VehicleState) -> Decision:
        """Landing is always allowed - except when the RC pilot has taken over (never fight the pilot)."""
        if (d := self._pilot(state)):
            return self._count(d)
        return self._count(Decision(True, value="LAND"))

    # ---------------- breach monitor -----------------
    def update_breach(self, state: VehicleState, now: Optional[float] = None) -> Optional[str]:
        """Call on every pose update. Returns a reason string the first time a breach must be acted on."""
        if self.locked or not state.armed or not state.in_air or not state.pos_fresh(self.limits.pose_timeout_s, now):
            self._breach_count = 0 if not self.locked else self._breach_count
            return None
        if self.pilot_control(state):
            # the RC pilot is flying: the fence is the pilot's responsibility, never take control away
            self._breach_count = 0
            return None
        why = self.fence.breached(state.pos)
        if self._rearm_needed:
            if why is None:
                self._rearm_needed = False
            return None
        if why is None:
            self._breach_count = 0
            return None
        self._breach_count += 1
        if self._breach_count >= self.rules.breach_samples:
            if self.rules.breach_action == "none":
                self._breach_count = 0   # report again after the next N samples, never lock
            else:
                self.lock(why)
            return why
        return None
