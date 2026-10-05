"""Fence geometry. Pure Python, no ROS imports, so it can be unit-tested anywhere.

Conventions (same as MAVROS):
  * local frame = ENU metres (x east, y north, z up)
  * body frame  = FLU (x forward, y left, z up)
  * yaw         = ENU yaw in radians, 0 = facing east, CCW positive

Global fences are evaluated in a flat local tangent plane (east/north metres)
around a reference point. For a lab field of a few hundred metres the error of
this approximation is far below a centimetre.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

EARTH_R = 6378137.0  # WGS84 equatorial radius [m]

Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def scale(self, k: float) -> "Vec3":
        return Vec3(self.x * k, self.y * k, self.z * k)

    def norm_xy(self) -> float:
        return math.hypot(self.x, self.y)

    def finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.x, self.y, self.z))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rotate_xy(x: float, y: float, yaw: float) -> Vec2:
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# --------------------------------------------------------------------------
# Rigid transform between the arena frame (what students and the fence use)
# and the MAVROS local frame (what the EKF uses). x' = R(yaw) * x + t
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Transform2D5:
    """Translation + yaw (z axis stays vertical). Identity by default."""

    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    yaw: float = 0.0

    @property
    def is_identity(self) -> bool:
        return self.tx == 0.0 and self.ty == 0.0 and self.tz == 0.0 and self.yaw == 0.0

    def point(self, p: Vec3) -> Vec3:
        x, y = rotate_xy(p.x, p.y, self.yaw)
        return Vec3(x + self.tx, y + self.ty, p.z + self.tz)

    def vector(self, v: Vec3) -> Vec3:
        x, y = rotate_xy(v.x, v.y, self.yaw)
        return Vec3(x, y, v.z)

    def heading(self, yaw: float) -> float:
        return wrap_pi(yaw + self.yaw)

    def inverse(self) -> "Transform2D5":
        # x = R^T (x' - t)
        ix, iy = rotate_xy(-self.tx, -self.ty, -self.yaw)
        return Transform2D5(ix, iy, -self.tz, -self.yaw)


# --------------------------------------------------------------------------
# Local tangent plane for lat/lon
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class LocalTangent:
    lat0: float
    lon0: float

    def to_en(self, lat: float, lon: float) -> Vec2:
        k = math.pi / 180.0
        east = (lon - self.lon0) * k * EARTH_R * math.cos(self.lat0 * k)
        north = (lat - self.lat0) * k * EARTH_R
        return (east, north)

    def to_latlon(self, east: float, north: float) -> Vec2:
        k = 180.0 / math.pi
        lat = self.lat0 + north / EARTH_R * k
        lon = self.lon0 + east / (EARTH_R * math.cos(math.radians(self.lat0))) * k
        return (lat, lon)


# --------------------------------------------------------------------------
# 2D helpers
# --------------------------------------------------------------------------
def point_in_polygon(p: Vec2, poly: Sequence[Vec2]) -> bool:
    """Even-odd rule. Points exactly on an edge count as inside."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if _on_segment(p, (x1, y1), (x2, y2)):
            return True
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xin:
                inside = not inside
    return inside


def _on_segment(p: Vec2, a: Vec2, b: Vec2, eps: float = 1e-9) -> bool:
    cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
    if abs(cross) > eps * max(1.0, math.dist(a, b)):
        return False
    return (min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps and
            min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps)


def dist_point_segment(p: Vec2, a: Vec2, b: Vec2) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def dist_to_polygon_edge(p: Vec2, poly: Sequence[Vec2]) -> float:
    return min(dist_point_segment(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def _seg_intersect(p1: Vec2, p2: Vec2, q1: Vec2, q2: Vec2) -> bool:
    def orient(a: Vec2, b: Vec2, c: Vec2) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2)
    d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0 and d3 != 0 and d4 != 0:
        return True
    return (_on_segment(p1, q1, q2) or _on_segment(p2, q1, q2) or
            _on_segment(q1, p1, p2) or _on_segment(q2, p1, p2))


def ray_distance_to_polygon(p: Vec2, d: Vec2, poly: Sequence[Vec2]) -> float:
    """Distance from p along unit direction d to the first polygon edge (inf if none)."""
    best = math.inf
    px, py = p
    dx, dy = d
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - px) * ey - (ay - py) * ex) / den   # along ray
        u = ((ax - px) * dy - (ay - py) * dx) / den   # along edge
        if t >= 0.0 and -1e-9 <= u <= 1.0 + 1e-9:
            best = min(best, t)
    return best


def ray_distance_to_circle(p: Vec2, d: Vec2, r: float) -> float:
    """p relative to circle centre, d unit direction. Distance to the circle along d."""
    b = p[0] * d[0] + p[1] * d[1]
    c = p[0] * p[0] + p[1] * p[1] - r * r
    disc = b * b - c
    if disc < 0:
        return math.inf
    s = math.sqrt(disc)
    for t in (-b - s, -b + s):
        if t >= 0:
            return t
    return math.inf


def offset_polygon_inward_ok(poly: Sequence[Vec2]) -> bool:
    """Sanity check used by config validation: polygon has >= 3 distinct vertices and non-zero area."""
    if len(poly) < 3:
        return False
    area = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        area += x1 * y2 - x2 * y1
    return abs(area) > 1e-6


def polygon_self_intersects(poly: Sequence[Vec2]) -> bool:
    n = len(poly)
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            b1, b2 = poly[j], poly[(j + 1) % n]
            if _seg_intersect(a1, a2, b1, b2):
                return True
    return False


def segment_leaves_polygon(a: Vec2, b: Vec2, poly: Sequence[Vec2]) -> bool:
    """True if the straight path a->b crosses the polygon boundary (both ends assumed inside)."""
    n = len(poly)
    for i in range(n):
        q1, q2 = poly[i], poly[(i + 1) % n]
        if _seg_intersect(a, b, q1, q2):
            # touching the boundary exactly at an endpoint is fine
            if _on_segment(a, q1, q2) or _on_segment(b, q1, q2):
                continue
            return True
    # also check the midpoint for collinear-overlap cases
    mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return not point_in_polygon(mid, poly)


def simple_stop_speed(distance: float, decel: float, lookahead: float) -> float:
    """Max speed toward a wall `distance` away so the drone can still stop before it.

    Stopping distance = v * T + v^2 / (2 a): it keeps going at v for the reaction time T
    (telemetry + command latency + ArduPilot's jerk-limited ramp into braking), then brakes at a.
    Solving for v gives v = -aT + sqrt((aT)^2 + 2 a d). Returns 0 at/over the wall.
    """
    if distance <= 0.0:
        return 0.0
    aT = decel * lookahead
    return -aT + math.sqrt(aT * aT + 2.0 * decel * distance)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def nearest_inside_box(p: Vec3, xmin: float, xmax: float, ymin: float, ymax: float,
                       zmin: float, zmax: float) -> Vec3:
    return Vec3(clamp(p.x, xmin, xmax), clamp(p.y, ymin, ymax), clamp(p.z, zmin, zmax))


def parse_polygon(raw: Sequence[Sequence[float]]) -> List[Vec2]:
    return [(float(a), float(b)) for a, b in raw]


def maybe_float(v) -> Optional[float]:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None
