import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dronelab.config import Config
from dronelab.sim.race import Course, Gate, Leaderboard, Race, fmt_time


def gate(yaw_deg=90):
    return Gate(0, 0.0, 10.0, 3.0, math.radians(yaw_deg), (1, 0, 0), 3.0, 3.0)


def test_gate_crossing_direction_and_opening():
    g = gate(90)  # fly north through (0, 10, 3)
    assert g.crossed((0, 9.5, 3), (0, 10.5, 3))
    assert not g.crossed((0, 10.5, 3), (0, 9.5, 3))        # wrong direction
    assert not g.crossed((2.0, 9.5, 3), (2.0, 10.5, 3))    # beside the frame (half width 1.5 + 0.25)
    assert not g.crossed((0, 9.5, 5.0), (0, 10.5, 5.0))    # above the frame
    assert g.crossed((1.6, 9.5, 3), (1.6, 10.5, 3))        # inside the tolerance


def test_course_file_is_consistent_with_config():
    c = Course.load()
    cfg = Config.load()
    prof = cfg.profiles["sim_arena"]
    assert prof["x"] == c.fence["x"] and prof["y"] == c.fence["y"] and prof["z"] == c.fence["z"]
    ds = cfg.drone(1)
    assert ds.profile_name == "sim_arena" and ds.arena_to_local.is_identity
    # every gate (its whole opening) must be inside the fence, or the race cannot be completed
    from dronelab.geo import Vec3
    for g in c.gates:
        for dz in (-g.height / 2, g.height / 2):
            assert ds.fence.check_point(Vec3(g.x, g.y, g.z + dz)) is None, g


def test_race_full_run(tmp_path):
    c = Course.load()
    r = Race(c)
    r.start(0.0)
    t = 0.0
    pos = (0.0, 0.0, 2.5)
    r.update(pos, False, t)
    events = []
    for g in c.gates:
        before = (g.x - 1.0 * math.cos(g.yaw), g.y - 1.0 * math.sin(g.yaw), g.z)
        after = (g.x + 1.0 * math.cos(g.yaw), g.y + 1.0 * math.sin(g.yaw), g.z)
        t += 5
        r.update(before, False, t)
        t += 0.1
        events += r.update(after, False, t)
    assert [e.kind for e in events][-1] == "all_gates"
    assert r.all_gates and not r.finished
    # landing outside the pad does not finish
    assert not r.update((10, 10, 0.1), True, t + 1)
    ev = r.update((0.5, 0.3, 0.05), True, t + 2)
    assert ev and ev[0].kind == "finish" and r.finished
    assert fmt_time(r.elapsed(99)) == fmt_time(t + 2)


def test_leaderboard(tmp_path):
    lb = Leaderboard(str(tmp_path / "lb.json"), size=3)
    assert lb.add("a", 30) == 1
    assert lb.add("b", 20) == 1
    assert lb.add("c", 40) == 3
    assert not lb.qualifies(50) and lb.qualifies(25)
    assert lb.add("d", 50) == 0
    assert [r["name"] for r in Leaderboard(str(tmp_path / "lb.json")).rows] == ["b", "a", "c"]
    assert lb.next_pilot_name() == "Pilot 1" and lb.next_pilot_name() == "Pilot 2"
