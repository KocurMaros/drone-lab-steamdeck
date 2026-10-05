import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from dronelab import geo
from dronelab.geo import Vec3
from dronelab.safety import (ACC_BITS, BoxFence, CircleFence, CommandGate, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED,
                             FRAME_LOCAL_NED, IGNORE_YAW, LANDED_IN_AIR, LANDED_ON_GROUND, Limits, POS_BITS,
                             PolygonFence, Rules, VEL_BITS, VehicleState, fence_from_config)

NOW = 1000.0


def st(x=1.0, y=1.0, z=1.5, yaw=0.0, armed=True, mode="GUIDED", landed=LANDED_IN_AIR):
    return VehicleState(pos=Vec3(x, y, z), yaw=yaw, pos_stamp=NOW, armed=armed, mode=mode, connected=True,
                        landed_state=landed)


def box_gate(**rules):
    return CommandGate(BoxFence(-0.5, 4.0, -0.5, 6.0, 0.5, 2.5, 0.3), Limits(), Rules(**rules))


# ---------------------------------------------------------------- geometry
def test_transform_roundtrip():
    t = geo.Transform2D5(1.0, -2.0, 0.5, math.radians(30))
    p = Vec3(3.0, 4.0, 1.0)
    q = t.inverse().point(t.point(p))
    assert q.x == pytest.approx(p.x) and q.y == pytest.approx(p.y) and q.z == pytest.approx(p.z)


def test_point_in_polygon_concave():
    # U shape (concave)
    poly = [(0, 0), (10, 0), (10, 10), (7, 10), (7, 3), (3, 3), (3, 10), (0, 10)]
    assert geo.point_in_polygon((1, 5), poly)
    assert geo.point_in_polygon((9, 5), poly)
    assert not geo.point_in_polygon((5, 5), poly)       # in the notch
    assert geo.point_in_polygon((0, 5), poly)           # on edge counts as inside
    assert geo.segment_leaves_polygon((1, 5), (9, 5), poly)   # straight line crosses the notch
    assert not geo.segment_leaves_polygon((1, 1), (9, 1), poly)


def test_ray_distances():
    sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert geo.ray_distance_to_polygon((5, 5), (1, 0), sq) == pytest.approx(5)
    assert geo.ray_distance_to_circle((0, 0), (0, 1), 7) == pytest.approx(7)
    assert geo.ray_distance_to_circle((3, 0), (1, 0), 5) == pytest.approx(2)


def test_tangent_plane_roundtrip():
    lt = geo.LocalTangent(48.15, 17.07)
    e, n = lt.to_en(48.151, 17.071)
    assert n == pytest.approx(111.3, rel=0.01)
    lat, lon = lt.to_latlon(e, n)
    assert lat == pytest.approx(48.151, abs=1e-9) and lon == pytest.approx(17.071, abs=1e-9)


# ---------------------------------------------------------------- box fence: positions
def test_box_accepts_inside_and_rejects_outside():
    g = box_gate()
    assert g.check_position(Vec3(1, 1, 1.5), st(), NOW).ok
    d = g.check_position(Vec3(5, 1, 1.5), st(), NOW)
    assert not d.ok and "x=5.00" in d.reason
    d = g.check_position(Vec3(1, 1, 0.2), st(), NOW)       # below the minimum command altitude
    assert not d.ok and "z=" in d.reason
    assert not g.check_position(Vec3(float("nan"), 1, 1), st(), NOW).ok
    assert g.stats == {"accepted": 1, "modified": 0, "rejected": 3}


def test_box_position_needs_no_pose():
    g = box_gate()
    s = VehicleState()  # nothing known yet
    assert g.check_position(Vec3(1, 1, 1), s, NOW).ok


# ---------------------------------------------------------------- box fence: velocities
def test_velocity_speed_limit():
    g = box_gate()
    d = g.check_velocity(Vec3(3.0, 0, 2.0), 3.0, st(x=1.5, y=3.0), NOW)
    v, yr = d.value
    assert d.ok and d.modified
    assert v.norm_xy() <= Limits().max_speed_xy + 1e-9
    assert abs(v.z) <= Limits().max_speed_z + 1e-9
    assert abs(yr) <= Limits().max_yaw_rate + 1e-9


def test_velocity_slows_to_zero_at_wall_and_allows_moving_away():
    g = box_gate()
    at_wall = st(x=4.0, y=3.0)
    v, _ = g.check_velocity(Vec3(0.8, 0, 0), 0, at_wall, NOW).value
    assert v.x == pytest.approx(0.0)
    v, _ = g.check_velocity(Vec3(-0.8, 0, 0), 0, at_wall, NOW).value
    assert v.x == pytest.approx(-0.8)
    # sliding along the wall keeps the tangential component
    v, _ = g.check_velocity(Vec3(0.5, 0.5, 0), 0, at_wall, NOW).value
    assert v.x == pytest.approx(0.0) and v.y == pytest.approx(0.5)


def test_velocity_slows_gradually_near_wall():
    g = box_gate()
    v_far, _ = g.check_velocity(Vec3(1.0, 0, 0), 0, st(x=1.0, y=3.0), NOW).value
    v_near, _ = g.check_velocity(Vec3(1.0, 0, 0), 0, st(x=3.5, y=3.0), NOW).value
    assert v_far.x == pytest.approx(1.0)
    assert 0 < v_near.x < 1.0


def test_velocity_outside_box_only_inward():
    g = box_gate()
    out = st(x=4.2, y=3.0)
    assert g.check_velocity(Vec3(0.5, 0, 0), 0, out, NOW).value[0].x == 0.0
    assert g.check_velocity(Vec3(-0.5, 0, 0), 0, out, NOW).value[0].x == pytest.approx(-0.5)


def test_velocity_without_pose_becomes_hover():
    g = box_gate()
    s = st()
    s.pos_stamp = NOW - 5.0
    d = g.check_velocity(Vec3(0.5, 0, 0), 0.1, s, NOW)
    assert not d.ok
    assert d.value == (Vec3(0, 0, 0), 0.0)  # caller must send hover, not drop


def test_floor_blocks_descent_velocity():
    g = box_gate()
    v, _ = g.check_velocity(Vec3(0, 0, -0.5), 0, st(z=0.5), NOW).value
    assert v.z == pytest.approx(0.0)


# ---------------------------------------------------------------- raw PositionTarget
def _mask(use_pos=False, use_vel=False):
    m = ACC_BITS | IGNORE_YAW
    if not use_pos:
        m |= POS_BITS
    if not use_vel:
        m |= VEL_BITS
    return m


def test_raw_body_velocity_rotates_and_clamps():
    g = box_gate()
    # facing +y (yaw 90 deg) at the +y wall: forward body velocity must be stopped
    s = st(x=1.0, y=6.0, yaw=math.pi / 2)
    d = g.check_raw_local(FRAME_BODY_NED, _mask(use_vel=True), Vec3(0, 0, 0), Vec3(0.8, 0, 0), 0.0, s, NOW)
    assert d.ok and d.modified
    assert d.value["vel"].x == pytest.approx(0.0, abs=1e-9)
    # backwards is fine
    d = g.check_raw_local(FRAME_BODY_NED, _mask(use_vel=True), Vec3(0, 0, 0), Vec3(-0.8, 0, 0), 0.0, s, NOW)
    assert d.value["vel"].x == pytest.approx(-0.8)


def test_raw_body_offset_position():
    g = box_gate()
    s = st(x=3.5, y=1.0, yaw=0.0)
    ok = g.check_raw_local(FRAME_BODY_OFFSET_NED, _mask(use_pos=True), Vec3(0.4, 0, 0), Vec3(0, 0, 0), 0, s, NOW)
    bad = g.check_raw_local(FRAME_BODY_OFFSET_NED, _mask(use_pos=True), Vec3(1.0, 0, 0), Vec3(0, 0, 0), 0, s, NOW)
    assert ok.ok and not bad.ok


def test_raw_rejects_accel_partial_and_bad_frames():
    g = box_gate()
    s = st()
    assert not g.check_raw_local(FRAME_LOCAL_NED, 0, Vec3(1, 1, 1), Vec3(0, 0, 0), 0, s, NOW).ok  # accel used
    partial = (_mask(use_pos=True) | 4)  # z ignored
    assert not g.check_raw_local(FRAME_LOCAL_NED, partial, Vec3(1, 1, 1), Vec3(0, 0, 0), 0, s, NOW).ok
    assert not g.check_raw_local(5, _mask(use_pos=True), Vec3(1, 1, 1), Vec3(0, 0, 0), 0, s, NOW).ok


# ---------------------------------------------------------------- services
def test_mode_rules():
    g = box_gate()
    assert not g.check_mode("GUIDED", st(armed=False)).ok
    assert g.check_mode("guided", st(armed=True)).ok
    assert not g.check_mode("ACRO", st()).ok
    g2 = box_gate(guided_requires_armed=False)
    assert g2.check_mode("GUIDED", st(armed=False)).ok


def test_arming_rules():
    g = box_gate()
    assert g.check_arm(True, st(armed=False, landed=LANDED_ON_GROUND)).value == "arm_sequence"
    assert box_gate(arm_sequence="direct").check_arm(True, st(armed=False)).value == "arm"
    assert g.check_arm(False, st(armed=True, landed=LANDED_IN_AIR)).value == "land_instead"
    assert g.check_arm(False, st(armed=True, landed=LANDED_ON_GROUND)).value == "disarm"


def test_takeoff_rules():
    g = box_gate()
    assert g.check_takeoff(1.5, st(armed=True, mode="GUIDED")).ok
    assert not g.check_takeoff(1.5, st(armed=False)).ok
    assert not g.check_takeoff(1.5, st(armed=True, mode="LOITER")).ok
    assert not g.check_takeoff(5.0, st(armed=True, mode="GUIDED")).ok


# ---------------------------------------------------------------- breach monitor
def test_breach_locks_after_n_samples_and_blocks_everything_but_land():
    g = box_gate(breach_samples=3)
    s = st(x=4.5, y=3.0)  # 0.5 m past the wall, margin 0.3
    assert g.update_breach(s, NOW) is None
    assert g.update_breach(s, NOW) is None
    why = g.update_breach(s, NOW)
    assert why and g.locked
    assert not g.check_position(Vec3(1, 1, 1), s, NOW).ok
    d = g.check_velocity(Vec3(-0.5, 0, 0), 0, s, NOW)
    assert not d.ok and d.value[0] == Vec3(0, 0, 0)
    assert not g.check_mode("GUIDED", s).ok
    assert g.check_mode("LAND", s).ok
    assert not g.check_arm(True, st(armed=False)).ok
    g.release()
    assert g.check_position(Vec3(1, 1, 1), s, NOW).ok
    # still outside after the release: not re-locked until it has been back inside once
    for _ in range(5):
        assert g.update_breach(s, NOW) is None
    assert g.update_breach(st(x=2.0, y=3.0), NOW) is None
    for _ in range(3):
        why = g.update_breach(s, NOW)
    assert why and g.locked


def test_breach_ignored_within_margin_on_ground_or_disarmed():
    g = box_gate(breach_samples=1)
    assert g.update_breach(st(x=4.2), NOW) is None                         # inside margin
    assert g.update_breach(st(x=6.0, landed=LANDED_ON_GROUND), NOW) is None  # on the ground
    assert g.update_breach(st(x=6.0, armed=False), NOW) is None
    assert g.update_breach(st(x=1.0, z=0.1), NOW) is None                  # below z_min is not a breach
    assert g.update_breach(st(x=1.0, z=3.0), NOW) is not None               # ceiling is


def test_breach_action_none_never_locks():
    g = box_gate(breach_samples=1, breach_action="none")
    assert g.update_breach(st(x=6.0), NOW)
    assert not g.locked


# ---------------------------------------------------------------- global fences
FIELD = [(48.1530, 17.0700), (48.1530, 17.0730), (48.1545, 17.0730), (48.1545, 17.0700)]


def test_polygon_fence_global():
    f = PolygonFence(points=FIELD, zmin=2, zmax=30, margin=2)
    g = CommandGate(f, Limits(max_speed_xy=5, max_speed_z=2), Rules(arm_sequence="direct"))
    c = f.to_plane(48.15375, 17.0715)
    s = VehicleState(pos=Vec3(c[0], c[1], 10), yaw=0, pos_stamp=NOW, armed=True, mode="GUIDED",
                     landed_state=LANDED_IN_AIR)
    inside = f.to_plane(48.1540, 17.0720)
    outside = f.to_plane(48.1560, 17.0720)
    assert g.check_position(Vec3(inside[0], inside[1], 10), s, NOW).ok
    assert not g.check_position(Vec3(outside[0], outside[1], 10), s, NOW).ok
    # AMSL altitude by mistake (Bratislava ~140 m) -> rejected
    d = g.check_position(Vec3(inside[0], inside[1], 140), s, NOW)
    assert not d.ok and "alt" in d.reason


def test_polygon_rejects_path_through_notch():
    poly = [(0, 0), (10, 0), (10, 10), (7, 10), (7, 3), (3, 3), (3, 10), (0, 10)]
    f = PolygonFence(points=poly, zmin=0.5, zmax=3, margin=0.3, is_global=False)
    g = CommandGate(f, Limits(), Rules())
    s = VehicleState(pos=Vec3(1, 8, 1), yaw=0, pos_stamp=NOW, armed=True, mode="GUIDED", landed_state=LANDED_IN_AIR)
    d = g.check_position(Vec3(9, 8, 1), s, NOW)
    assert not d.ok and "path" in d.reason
    assert g.check_position(Vec3(1, 1, 1), s, NOW).ok


def test_polygon_velocity_stops_at_edge():
    f = PolygonFence(points=[(0, 0), (10, 0), (10, 10), (0, 10)], zmin=0.5, zmax=3, margin=0.3, is_global=False)
    g = CommandGate(f, Limits(), Rules())
    s = VehicleState(pos=Vec3(10, 5, 1), yaw=0, pos_stamp=NOW, armed=True, mode="GUIDED", landed_state=LANDED_IN_AIR)
    v, _ = g.check_velocity(Vec3(0.8, 0, 0), 0, s, NOW).value
    assert v.x == pytest.approx(0.0, abs=1e-6)


def test_circle_fence():
    f = CircleFence(lat=48.15, lon=17.07, radius=50, zmin=2, zmax=30, margin=2)
    g = CommandGate(f, Limits(), Rules())
    s = VehicleState(pos=Vec3(0, 0, 10), yaw=0, pos_stamp=NOW, armed=True, mode="GUIDED", landed_state=LANDED_IN_AIR)
    assert g.check_position(Vec3(30, 30, 10), s, NOW).ok
    assert not g.check_position(Vec3(40, 40, 10), s, NOW).ok
    s.pos = Vec3(53, 0, 10)
    assert g.update_breach(s, NOW) is None  # within margin
    s.pos = Vec3(60, 0, 10)
    g.rules.breach_samples = 1
    assert g.update_breach(s, NOW)


def test_fence_from_config_validates():
    assert isinstance(fence_from_config({"type": "box", "x": [0, 1], "y": [0, 1], "z": [0.2, 1]}), BoxFence)
    with pytest.raises(ValueError):
        fence_from_config({"type": "box", "x": [1, 0], "y": [0, 1], "z": [0.2, 1]})
    with pytest.raises(ValueError):
        fence_from_config({"type": "polygon", "frame": "local", "points": [[0, 0], [1, 1], [0, 1], [1, 0]]})
    with pytest.raises(ValueError):
        fence_from_config({"type": "hexagon"})


# ---------------------------------------------------------------- review fixes
from dronelab.safety import HOVER_RAW, LANDED_UNDEFINED, VEL_BITS as _VB  # noqa: E402


def test_feed_forward_velocity_is_dropped_from_position_targets():
    g = box_gate()
    m = ACC_BITS | IGNORE_YAW   # position AND velocity used
    d = g.check_raw_local(FRAME_LOCAL_NED, m, Vec3(3.9, 3, 2.4), Vec3(1, 0, 0.5), 0, st(), NOW)
    assert d.ok and d.modified
    assert d.value["type_mask"] & _VB == _VB and d.value["vel"] == Vec3(0, 0, 0)


def test_rejected_velocity_like_raw_commands_become_hover():
    g = box_gate()
    s = st()
    partial_vel = ACC_BITS | IGNORE_YAW | POS_BITS | 32      # vz ignored
    for frame, mask in ((FRAME_LOCAL_NED, partial_vel), (5, ACC_BITS | IGNORE_YAW | POS_BITS),
                        (FRAME_LOCAL_NED, IGNORE_YAW | POS_BITS)):   # last one: acceleration used
        d = g.check_raw_local(frame, mask, Vec3(0, 0, 0), Vec3(1, 0, 0), 0, s, NOW)
        assert not d.ok and d.value == HOVER_RAW, (frame, mask)
    g.lock("test")
    d = g.check_raw_local(FRAME_BODY_NED, _mask(use_vel=True), Vec3(0, 0, 0), Vec3(1, 0, 0), 0, s, NOW)
    assert not d.ok and d.value == HOVER_RAW
    # rejected POSITION commands are simply dropped (ArduPilot holds the last target)
    g.release()
    d = g.check_raw_local(FRAME_LOCAL_NED, _mask(use_pos=True), Vec3(9, 9, 1), Vec3(0, 0, 0), 0, s, NOW)
    assert not d.ok and d.value is None


def test_students_cannot_leave_instructor_or_failsafe_modes():
    g = box_gate()
    s = st(mode="LAND", armed=True)
    assert not g.check_mode("GUIDED", s).ok          # LAND chosen by instructor/failsafe
    g.student_mode = "LAND"
    assert g.check_mode("GUIDED", s).ok              # the student's own landing may be aborted
    g.student_mode = ""
    assert g.check_mode("GUIDED", st(mode="LAND", armed=False)).ok is False  # GUIDED needs armed anyway
    assert g.check_mode("LOITER", st(mode="LAND", armed=False)).ok           # on the ground: fine


def test_default_modes_exclude_rtl_and_always_include_land():
    g = box_gate()
    assert not g.check_mode("RTL", st()).ok
    assert "LAND" in Rules(allowed_modes=["GUIDED"]).allowed_modes


def test_in_air_without_extended_state():
    s = st(landed=LANDED_UNDEFINED, z=0.1)
    assert not s.in_air
    s = st(landed=LANDED_UNDEFINED, z=1.0)
    assert s.in_air
    s = VehicleState(armed=True)
    assert s.in_air                         # no height known: assume flying
    assert not VehicleState(armed=False, landed_state=LANDED_IN_AIR).in_air


def test_breach_not_triggered_on_ground_without_extended_state():
    g = box_gate(breach_samples=1)
    assert g.update_breach(st(x=6.0, z=0.05, landed=LANDED_UNDEFINED), NOW) is None


def test_config_validation(tmp_path):
    import yaml
    from dronelab.config import Config, DEFAULT_CONFIG
    data = yaml.safe_load(open(DEFAULT_CONFIG))
    cfg = Config(data)
    assert "RTL" not in cfg.drone(11).rules.allowed_modes
    assert "RTL" in cfg.drone(11, "outdoor_field").rules.allowed_modes
    data["limits"]["decel"] = -1
    with pytest.raises(ValueError):
        Config(data).drone(11)
    with pytest.raises(ValueError):
        Config(yaml.safe_load(open(DEFAULT_CONFIG))).drone(40)   # private domain 110 > 101


def test_stop_buffer_stops_inside_the_fence():
    g = CommandGate(BoxFence(-0.5, 4.0, -0.5, 6.0, 0.5, 2.5, 0.3), Limits(stop_buffer_m=0.5), Rules())
    v, _ = g.check_velocity(Vec3(1.0, 0, 0), 0, st(x=3.5, y=3.0), NOW).value
    assert v.x == pytest.approx(0.0)
    v, _ = g.check_velocity(Vec3(1.0, 0, 0), 0, st(x=3.0, y=3.0), NOW).value
    assert 0 < v.x < 1.0


def test_stop_speed_model():
    # v*T + v^2/(2a) == d at the returned speed
    for d in (0.3, 1.0, 5.0, 15.0):
        v = geo.simple_stop_speed(d, 3.0, 1.5)
        assert v * 1.5 + v * v / 6.0 == pytest.approx(d)
