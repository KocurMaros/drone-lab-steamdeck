"""Gate race logic for the demo (pure Python, unit-tested)."""
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import yaml

from ..config import REPO_ROOT, VAR_DIR

COURSE_FILE = os.path.join(REPO_ROOT, "sim", "course", "arena.yaml")


@dataclass
class Gate:
    idx: int
    x: float
    y: float
    z: float
    yaw: float            # radians, ENU heading to fly through
    color: Tuple[float, float, float]
    width: float
    height: float

    def to_local(self, px: float, py: float, pz: float) -> Tuple[float, float, float]:
        """Point -> gate frame: (along the flight direction, left, up) relative to the gate centre."""
        dx, dy, dz = px - self.x, py - self.y, pz - self.z
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy, -s * dx + c * dy, dz)

    def crossed(self, a: Tuple[float, float, float], b: Tuple[float, float, float], margin: float = 0.25) -> bool:
        """True if the segment a->b passes through the opening in the forward direction."""
        la, lb = self.to_local(*a), self.to_local(*b)
        if not (la[0] < 0.0 <= lb[0]):
            return False
        t = -la[0] / (lb[0] - la[0])
        lat = la[1] + t * (lb[1] - la[1])
        up = la[2] + t * (lb[2] - la[2])
        return abs(lat) <= self.width / 2 + margin and abs(up) <= self.height / 2 + margin

    @property
    def qcolor_hex(self) -> str:
        r, g, b = (int(255 * c) for c in self.color)
        return f"#{r:02x}{g:02x}{b:02x}"


@dataclass
class Course:
    name: str
    gates: List[Gate]
    pad: Tuple[float, float, float]   # x, y, radius
    fence: dict
    buildings: list
    trees: list
    towers: list
    roads: list

    @classmethod
    def load(cls, path: str = COURSE_FILE) -> "Course":
        c = yaml.safe_load(open(path, encoding="utf-8"))
        gs = c.get("gate_size", {"width": 3.0, "height": 3.0})
        gates = [Gate(i, float(g["x"]), float(g["y"]), float(g["z"]), math.radians(float(g["yaw"])),
                      tuple(g.get("color", (1, 0.5, 0))), float(gs["width"]), float(gs["height"]))
                 for i, g in enumerate(c["gates"])]
        p = c.get("pad", {"x": 0, "y": 0, "radius": 2})
        return cls(c.get("name", "course"), gates, (float(p["x"]), float(p["y"]), float(p.get("radius", 2.0))),
                   c.get("fence", {}), c.get("buildings", []), c.get("trees", []), c.get("towers", []),
                   c.get("roads", []))


@dataclass
class RaceEvent:
    kind: str     # "gate", "all_gates", "finish", "start"
    text: str
    gate: int = -1


@dataclass
class Race:
    course: Course
    active: bool = False
    next_gate: int = 0
    t_start: Optional[float] = None
    t_end: Optional[float] = None
    splits: List[float] = field(default_factory=list)
    _last: Optional[Tuple[float, float, float]] = None

    def reset(self):
        self.active = False
        self.next_gate = 0
        self.t_start = None
        self.t_end = None
        self.splits = []
        self._last = None

    def start(self, now: float):
        self.reset()
        self.active = True
        self.t_start = now

    @property
    def finished(self) -> bool:
        return self.t_end is not None

    @property
    def all_gates(self) -> bool:
        return self.next_gate >= len(self.course.gates)

    def elapsed(self, now: float) -> float:
        if self.t_start is None:
            return 0.0
        return (self.t_end if self.t_end is not None else now) - self.t_start

    def update(self, pos: Tuple[float, float, float], landed: bool, now: float) -> List[RaceEvent]:
        ev: List[RaceEvent] = []
        last, self._last = self._last, pos
        if not self.active or self.finished or last is None:
            return ev
        if not self.all_gates:
            g = self.course.gates[self.next_gate]
            if g.crossed(last, pos):
                self.splits.append(self.elapsed(now))
                self.next_gate += 1
                if self.all_gates:
                    ev.append(RaceEvent("all_gates", "ALL GATES! Now land on the pad", g.idx))
                else:
                    ev.append(RaceEvent("gate", f"GATE {g.idx + 1}", g.idx))
        elif landed:
            px, py, r = self.course.pad
            if math.hypot(pos[0] - px, pos[1] - py) <= r:
                self.t_end = now
                ev.append(RaceEvent("finish", f"FINISHED {fmt_time(self.elapsed(now))}"))
        return ev


def fmt_time(t: float) -> str:
    m, s = divmod(max(0.0, t), 60)
    return f"{int(m):02d}:{s:05.2f}"


class Leaderboard:
    def __init__(self, path: Optional[str] = None, size: int = 10):
        self.path = path or os.path.join(VAR_DIR, "leaderboard.json")
        self.size = size
        self.rows: List[dict] = []
        self.load()

    def load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                self.rows = json.load(f)
        except (OSError, ValueError):
            self.rows = []

    def save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.rows, f, indent=1)
        os.replace(tmp, self.path)

    def qualifies(self, t: float) -> bool:
        return len(self.rows) < self.size or t < self.rows[-1]["time"]

    def add(self, name: str, t: float) -> int:
        """Returns the 1-based rank (0 if it did not make the board)."""
        row = {"name": name[:16], "time": round(t, 2), "date": time.strftime("%Y-%m-%d %H:%M")}
        self.rows.append(row)
        self.rows.sort(key=lambda r: r["time"])
        self.rows = self.rows[: self.size]
        self.save()
        return self.rows.index(row) + 1 if row in self.rows else 0

    def next_pilot_name(self) -> str:
        n = 1
        try:
            n = int(open(self.path + ".count").read().strip()) + 1
        except (OSError, ValueError):
            pass
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            open(self.path + ".count", "w").write(str(n))
        except OSError:
            pass
        return f"Pilot {n}"

    def clear(self):
        self.rows = []
        self.save()
