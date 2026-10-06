"""ArduCopter parameters that matter for RC takeover and fail-safes.

The gate reads these from the drone (through MAVROS) after connecting and the Flight app shows
what is wrong; ``fixes()`` gives the exact values the "Fix parameters" button writes.

What "RC takeover" needs on the flight controller:
  * RC_OPTIONS bit 1 (value 2) "Ignore MAVLink RC overrides" - only the real transmitter moves
    the drone by sticks / mode switch; nothing on the network can override your RC.
  * a LOITER (or POSHOLD/ALT_HOLD) position on the flight-mode switch. ArduPilot only reacts to
    switch CHANGES: if the switch already sits on LOITER while the drone flies GUIDED, flick it
    to another position and back.
  * FS_GCS_ENABLE != 0 with FS_OPTIONS bit 4 (16): if the Deck/Wi-Fi dies during a GUIDED flight
    the drone RTLs (outdoor) or LANDs (indoor), but a pilot flying LOITER is NOT interrupted.
  * SYSID_MYGCS = the system id MAVROS uses (255), otherwise the GCS failsafe ignores the Deck.
The gate itself never fights the pilot: once the drone is in a mode the pilot chose, student
commands are rejected and the fence monitor stays quiet until the drone is back in GUIDED.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List

NAMES = ("RC_OPTIONS", "FLTMODE_CH", "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5", "FLTMODE6",
         "FS_GCS_ENABLE", "FS_OPTIONS", "FS_GCS_TIMEOUT", "FS_THR_ENABLE", "FENCE_ENABLE", "FENCE_TYPE",
         "FENCE_ALT_MAX", "FENCE_RADIUS", "FENCE_ACTION", "RTL_ALT", "WPNAV_SPEED", "SYSID_MYGCS",
         "EK3_SRC1_POSZ")

REQUIRED = ("RC_OPTIONS", "FLTMODE1", "FS_GCS_ENABLE", "FS_OPTIONS", "SYSID_MYGCS")   # all ArduCopter versions

MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
         9: "LAND", 11: "DRIFT", 13: "SPORT", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
         20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 27: "AUTO_RTL"}
PILOT_MODES = {0, 2, 5, 16}            # STABILIZE, ALT_HOLD, LOITER, POSHOLD: the pilot flies with sticks
RC_IGNORE_OVERRIDES = 2                # RC_OPTIONS bit 1
FS_CONTINUE_PILOT = 16                 # FS_OPTIONS bit 4: GCS failsafe does not interrupt pilot modes
FS_GCS_RTL, FS_GCS_LAND = 1, 5
FS_THR_RTL, FS_THR_LAND = 1, 3


@dataclass
class Finding:
    level: str                          # "warn" (should be fixed) | "info"
    text: str
    fix: Dict[str, float] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"level": self.level, "text": self.text, "fix": dict(self.fix)}


def _i(p: Dict[str, float], name: str) -> int:
    return int(round(p[name]))


def fence_extent(fence) -> float:
    """Largest distance between two points of the fence (m). Any take-off point inside the fence is
    then at most this far from any other point inside it - a safe radius for ArduPilot's circle fence."""
    poly = getattr(fence, "poly", None)
    if poly:
        return max(math.dist(a, b) for a in poly for b in poly)
    if hasattr(fence, "radius"):
        return 2.0 * fence.radius
    if hasattr(fence, "xmin"):
        return math.hypot(fence.xmax - fence.xmin, fence.ymax - fence.ymin)
    return 100.0


def review(p: Dict[str, float], ds) -> List[Finding]:
    """Check the drone's parameters ``p`` (name -> value) against the drone settings ``ds``."""
    out: List[Finding] = []
    outdoor = ds.fence.kind == "global"
    ceiling = float(ds.fence.zmax)

    if "RC_OPTIONS" in p:
        v = _i(p, "RC_OPTIONS")
        if not v & RC_IGNORE_OVERRIDES:
            out.append(Finding("warn", f"RC_OPTIONS={v}: MAVLink RC overrides are accepted, so a program on the network "
                                       f"can override your transmitter. Fix: RC_OPTIONS={v | RC_IGNORE_OVERRIDES} "
                                       f"(bit 1 'ignore MAVLink overrides').",
                               {"RC_OPTIONS": v | RC_IGNORE_OVERRIDES}))

    slots = {i: _i(p, f"FLTMODE{i}") for i in range(1, 7) if f"FLTMODE{i}" in p}
    if slots:
        names = ", ".join(f"{i}={MODES.get(m, m)}" for i, m in slots.items())
        if not set(slots.values()) & PILOT_MODES:
            out.append(Finding("warn", f"No flight-mode switch position is LOITER/POSHOLD/ALT_HOLD ({names}): "
                                       f"you cannot take over from the RC. Put LOITER on a switch position."))
        else:
            ch = _i(p, "FLTMODE_CH") if "FLTMODE_CH" in p else "?"
            out.append(Finding("info", f"Takeover: flight-mode switch on channel {ch} ({names}). ArduPilot only reacts "
                                       f"to switch CHANGES - if the switch already sits on LOITER while the drone flies "
                                       f"GUIDED, flick it to another position and back."))

    if "FS_GCS_ENABLE" in p:
        v = _i(p, "FS_GCS_ENABLE")
        want = FS_GCS_RTL if outdoor else FS_GCS_LAND
        opts = _i(p, "FS_OPTIONS") if "FS_OPTIONS" in p else 0
        if v == 0:
            fix = {"FS_GCS_ENABLE": want}
            if "FS_OPTIONS" in p and not opts & FS_CONTINUE_PILOT:
                fix["FS_OPTIONS"] = opts | FS_CONTINUE_PILOT
            out.append(Finding("warn", f"FS_GCS_ENABLE=0: if the Deck or Wi-Fi dies during a GUIDED flight the drone "
                                       f"keeps hovering at its last target. Fix: FS_GCS_ENABLE={want} "
                                       f"({'RTL' if outdoor else 'LAND'}) and FS_OPTIONS={opts | FS_CONTINUE_PILOT} "
                                       f"(a pilot flying LOITER is not interrupted).", fix))
        elif "FS_OPTIONS" in p and not opts & FS_CONTINUE_PILOT:
            out.append(Finding("warn", f"FS_OPTIONS={opts}: losing the Deck would also interrupt the RC pilot flying "
                                       f"LOITER. Fix: FS_OPTIONS={opts | FS_CONTINUE_PILOT} "
                                       f"(continue in pilot-controlled modes).",
                               {"FS_OPTIONS": opts | FS_CONTINUE_PILOT}))

    if "SYSID_MYGCS" in p and _i(p, "SYSID_MYGCS") != int(ds.mavros_system_id):
        out.append(Finding("warn", f"SYSID_MYGCS={_i(p, 'SYSID_MYGCS')} but MAVROS uses system id "
                                   f"{ds.mavros_system_id}: the GCS failsafe does not see the Deck.",
                           {"SYSID_MYGCS": int(ds.mavros_system_id)}))

    if "FS_THR_ENABLE" in p and _i(p, "FS_THR_ENABLE") == 0:
        want = FS_THR_RTL if outdoor else FS_THR_LAND
        out.append(Finding("warn", f"FS_THR_ENABLE=0: no failsafe when the transmitter is lost. "
                                   f"Fix: FS_THR_ENABLE={want} ({'RTL' if outdoor else 'LAND'}).",
                           {"FS_THR_ENABLE": want}))

    if "FENCE_ENABLE" in p and _i(p, "FENCE_ENABLE") == 0:
        if outdoor:
            radius = max(30.0, math.ceil((fence_extent(ds.fence) + 10.0) / 10.0) * 10.0)
            alt = math.ceil(ceiling + 10.0)
            out.append(Finding("warn", f"FENCE_ENABLE=0: ArduPilot's own fence is off, so nothing limits the drone if "
                                       f"the Deck dies or the pilot flies away. Fix: a backstop circle fence around the "
                                       f"take-off point, larger than the gate's fence (radius {radius:.0f} m, "
                                       f"max altitude {alt:.0f} m, action RTL).",
                               {"FENCE_ENABLE": 1, "FENCE_TYPE": 3, "FENCE_RADIUS": radius, "FENCE_ALT_MAX": alt,
                                "FENCE_ACTION": 1}))
        else:
            out.append(Finding("info", "FENCE_ENABLE=0: indoors the gate's box is the only fence (normal without GPS)."))

    if "RTL_ALT" in p and p["RTL_ALT"] / 100.0 > ceiling:
        if outdoor:
            want = int(max(ceiling - 2.0, 3.0) * 100)
            out.append(Finding("warn", f"RTL_ALT={p['RTL_ALT'] / 100:.1f} m is above the fence ceiling {ceiling:g} m: "
                                       f"RTL would climb out of the fence. Fix: RTL_ALT={want} ({want / 100:g} m).",
                               {"RTL_ALT": want}))
        else:
            out.append(Finding("info", f"RTL_ALT={p['RTL_ALT'] / 100:.1f} m is above the indoor ceiling: RTL is not "
                                       f"allowed indoors (failsafes should LAND: FS_GCS_ENABLE=5, FS_THR_ENABLE=3)."))

    if not outdoor and "EK3_SRC1_POSZ" in p and _i(p, "EK3_SRC1_POSZ") == 1:
        out.append(Finding("warn", "EK3_SRC1_POSZ=1: the height comes from the barometer, which drifts indoors (often "
                                   "1-2 m within 20 min). The fence floor/ceiling and take-off heights drift with it - "
                                   "a drone on the floor can read z = 2 m. With OptiTrack use EK3_SRC1_POSZ=6 "
                                   "(ExternalNav); change it in Mission Planner and test before flying students."))

    if "WPNAV_SPEED" in p and p["WPNAV_SPEED"] / 100.0 > ds.limits.max_speed_xy * 1.5:
        want = int(round(ds.limits.max_speed_xy * 100))
        out.append(Finding("warn",
                           f"WPNAV_SPEED={p['WPNAV_SPEED'] / 100:.1f} m/s: student POSITION targets (and RTL) fly "
                           f"this fast; the gate limits only velocity commands ({ds.limits.max_speed_xy:g} m/s). "
                           f"Fix: WPNAV_SPEED={want}.", {"WPNAV_SPEED": want}))
    return out


def fixes(findings: List[Finding]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for f in findings:
        out.update(f.fix)
    return out


def load_param_file(path: str) -> Dict[str, float]:
    """Mission Planner / MAVProxy ``NAME,VALUE`` (or ``NAME VALUE``) files; '#' starts a comment.
    QGroundControl files (``sysid compid NAME VALUE type``) are read too."""
    out: Dict[str, float] = {}
    with open(path) as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").replace("\t", " ").split()
            if len(parts) >= 5 and parts[0].isdigit() and parts[1].isdigit():
                parts = parts[2:4]
            if len(parts) < 2:
                raise ValueError(f"{path}: cannot read line: {raw.strip()}")
            out[parts[0].upper()] = float(parts[1])
    return out
