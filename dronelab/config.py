"""Configuration loading (config/dronelab.yaml + optional config/local.yaml)."""
from __future__ import annotations

import copy
import ipaddress
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml

from . import geo
from .safety import Fence, Limits, Rules, fence_from_config

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONFIG_DIR = os.path.join(REPO_ROOT, "config")
DEFAULT_CONFIG = os.path.join(CONFIG_DIR, "dronelab.yaml")
LOCAL_CONFIG = os.path.join(CONFIG_DIR, "local.yaml")
VAR_DIR = os.environ.get("DRONELAB_VAR", os.path.join(REPO_ROOT, "var"))
RUN_DIR = os.environ.get("DRONELAB_RUN", "/tmp/dronelab")


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def parse_ports(items) -> List[int]:
    ports = set()
    for it in items or []:
        s = str(it).strip()
        if "-" in s:
            a, b = s.split("-", 1)
            ports.update(range(int(a), int(b) + 1))
        elif s:
            ports.add(int(s))
    bad = [p for p in ports if not 1 <= p <= 65535]
    if bad:
        raise ValueError(f"invalid UDP ports: {bad}")
    return sorted(ports)


def parse_targets(items) -> List[str]:
    """'192.168.18.110-119', '10.42.0.1', '192.168.18.0/24' -> list of IPv4 strings."""
    out: List[str] = []
    for it in items or []:
        s = str(it).strip()
        if not s:
            continue
        if "/" in s:
            net = ipaddress.ip_network(s, strict=False)
            if net.num_addresses > 1024:
                raise ValueError(f"probe target {s} is too large (max /22)")
            out.extend(str(h) for h in net.hosts())
        elif "-" in s:
            base, last = s.rsplit("-", 1)
            first = ipaddress.ip_address(base)
            prefix = str(first).rsplit(".", 1)[0]
            for i in range(int(str(first).rsplit(".", 1)[1]), int(last) + 1):
                out.append(f"{prefix}.{i}")
        else:
            out.append(str(ipaddress.ip_address(s)))
    seen, uniq = set(), []
    for ip in out:
        if ip not in seen:
            seen.add(ip)
            uniq.append(ip)
    return uniq


@dataclass
class DroneSettings:
    sysid: int
    student_ns: str
    private_domain: int
    student_domain: int
    profile_name: str
    profile_label: str
    profile_verified: bool
    fence: Fence
    arena_to_local: geo.Transform2D5
    limits: Limits
    rules: Rules
    mavros_launch: str
    mavros_system_id: int
    gcs_url: str
    error_rate_hz: float
    status_rate_hz: float
    student_interface: str

    @property
    def socket_path(self) -> str:
        return os.path.join(RUN_DIR, f"gate-{self.sysid}.sock")


class Config:
    def __init__(self, data: dict, path: str = DEFAULT_CONFIG):
        self.data = data
        self.path = path
        net = data.get("network", {})
        self.listen_ports = parse_ports(net.get("listen_ports", ["14510-14519", "14540-14559"]))
        self.probe_port = int(net.get("probe_port", 14550))
        self.probe_targets = parse_targets(net.get("probe_targets", []))
        self.probe_local_subnets = bool(net.get("probe_local_subnets", True))
        self.probe_interval_s = float(net.get("probe_interval_s", 3.0))
        self.stale_after_s = float(net.get("stale_after_s", 4.0))
        self.student_domain = int(net.get("student_domain", 0))
        self.private_domain_base = int(net.get("private_domain_base", 70))
        self.student_interface = str(net.get("student_interface", "auto"))
        self.profiles: Dict[str, dict] = data.get("fence_profiles", {})
        if not self.profiles:
            raise ValueError("config has no fence_profiles")
        # validate every profile up-front so a typo is caught when the app starts, not mid-flight
        for name in self.profiles:
            self._build_fence(name)

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, path: Optional[str] = None) -> "Config":
        path = path or os.environ.get("DRONELAB_CONFIG", DEFAULT_CONFIG)
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        local = os.path.join(os.path.dirname(path), "local.yaml")
        if os.path.exists(local):
            with open(local, "r", encoding="utf-8") as f:
                data = deep_merge(data, yaml.safe_load(f) or {})
        return cls(data, path)

    def profile_names(self) -> List[str]:
        return list(self.profiles.keys())

    def _build_fence(self, name: str) -> Fence:
        if name not in self.profiles:
            raise ValueError(f"unknown fence profile '{name}' (have: {', '.join(self.profiles)})")
        try:
            return fence_from_config(self.profiles[name])
        except (KeyError, TypeError, ValueError, IndexError) as e:
            raise ValueError(f"fence profile '{name}': {e}") from e

    def drone(self, sysid: int, profile: Optional[str] = None) -> DroneSettings:
        if not 1 <= sysid <= 255:
            raise ValueError("MAVLink system id must be 1..255")
        d = dict(self.data.get("drone_defaults", {}))
        over = (self.data.get("drone_overrides") or {}).get(sysid) or \
            (self.data.get("drone_overrides") or {}).get(str(sysid)) or {}
        d.update(over)
        prof_name = profile or d.get("fence_profile", "indoor_lab")
        prof = self.profiles.get(prof_name)
        fence = self._build_fence(prof_name)

        lim_cfg = dict(self.data.get("limits", {}))
        lim_cfg.update(prof.get("limits", {}) or {})
        limits = Limits(
            max_speed_xy=float(lim_cfg.get("max_speed_xy", 1.0)),
            max_speed_z=float(lim_cfg.get("max_speed_z", 0.5)),
            max_yaw_rate=math.radians(float(lim_cfg.get("max_yaw_rate_deg", 45))),
            lookahead_s=float(lim_cfg.get("lookahead_s", 1.0)),
            decel=float(lim_cfg.get("decel", 1.5)),
            pose_timeout_s=float(lim_cfg.get("pose_timeout_s", 0.5)),
            stop_buffer_m=float(lim_cfg.get("stop_buffer_m", 0.2)),
        )
        for k, v in vars(limits).items():
            if not (math.isfinite(v) and (v > 0 or (k == "stop_buffer_m" and v == 0))):
                raise ValueError(f"limits.{k} must be a positive number (profile '{prof_name}')")
        g = self.data.get("gate", {})
        action = str(g.get("breach_action", "land")).lower()
        if action not in ("land", "brake", "none"):
            raise ValueError("gate.breach_action must be land, brake or none")
        rules = Rules(
            arm_sequence=str(d.get("arm_sequence", "loiter_arm_guided")),
            guided_requires_armed=bool(d.get("guided_requires_armed", True)),
            allowed_modes=[str(m).upper() for m in prof.get("allowed_modes",
                                                            g.get("allowed_modes", Rules().allowed_modes))],
            breach_action=action,
            breach_samples=max(1, int(g.get("breach_samples", 3))),
        )
        a2l = prof.get("arena_to_local", {}) or {}
        tf = geo.Transform2D5(float(a2l.get("x", 0)), float(a2l.get("y", 0)), float(a2l.get("z", 0)),
                              math.radians(float(a2l.get("yaw_deg", 0))))
        private = self.private_domain_base + sysid
        if private > 101 or private == self.student_domain:
            raise ValueError(f"private ROS domain {private} for sysid {sysid} is invalid; "
                             f"lower network.private_domain_base")
        return DroneSettings(
            sysid=sysid,
            student_ns=str(d.get("student_ns", "/drone{sysid}")).format(sysid=sysid).rstrip("/"),
            private_domain=private,
            student_domain=self.student_domain,
            profile_name=prof_name,
            profile_label=str(prof.get("label", prof_name)),
            profile_verified=bool(prof.get("verified", False)),
            fence=fence,
            arena_to_local=tf,
            limits=limits,
            rules=rules,
            mavros_launch=str(d.get("mavros_launch", "apm")),
            mavros_system_id=int(d.get("mavros_system_id", 255)),
            gcs_url=str(d.get("gcs_url", "") or ""),
            error_rate_hz=float(g.get("error_rate_hz", 2.0)),
            status_rate_hz=float(g.get("status_rate_hz", 2.0)),
            student_interface=self.student_interface,
        )
