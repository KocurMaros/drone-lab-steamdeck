"""ROS 2 / DDS environment for the two-domain isolation.

MAVROS runs on a private domain that CycloneDDS binds to the loopback interface
only; students on the lab network cannot discover or talk to it. The gate
process is a member of both domains, so it gets a CycloneDDS configuration with
one <Domain> section per domain.
"""
from __future__ import annotations

import ipaddress
import json
import os
import subprocess
from typing import Dict, List, Optional, Tuple

from .config import RUN_DIR

LOOPBACK_DOMAIN = """
  <Domain Id="{domain}">
    <General>
      <Interfaces><NetworkInterface address="127.0.0.1" multicast="true"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers><Peer Address="127.0.0.1"/></Peers>
    </Discovery>
  </Domain>"""

# Students' domain: multicast discovery as usual, plus well-known unicast ports (participant index)
# so student PCs on networks without multicast can list the Deck as a unicast peer.
NETWORK_DOMAIN = """
  <Domain Id="{domain}">
    <General>{interfaces}</General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>"""


# container/VM bridges: up and multicast-capable, but students are never behind them
VIRTUAL_PREFIXES = ("docker", "podman", "cni", "veth", "virbr", "br-", "vmnet", "vboxnet", "lxc", "lxd", "flannel",
                    "tailscale", "zt", "wg", "tun", "tap")


def list_ipv4_interfaces() -> List[dict]:
    """[{name, ip, prefix, flags, state}] from `ip -j -4 addr` (iproute2 is in the image)."""
    try:
        js = subprocess.run(["ip", "-j", "-4", "addr", "show"], capture_output=True, text=True, timeout=2).stdout
        out = []
        for ifc in json.loads(js or "[]"):
            for a in ifc.get("addr_info", []):
                if a.get("family") == "inet":
                    out.append({"name": ifc.get("ifname", ""), "ip": a.get("local", ""), "prefix": a.get("prefixlen", 0),
                                "flags": list(ifc.get("flags", [])), "state": ifc.get("operstate", "")})
        return out
    except Exception:
        return []


def default_route_interface() -> Optional[str]:
    try:
        with open("/proc/net/route") as f:
            next(f)
            for line in f:
                cols = line.split()
                if len(cols) > 7 and cols[1] == "00000000" and cols[7] == "00000000":
                    return cols[0]
    except (OSError, StopIteration):
        pass
    return None


def choose_student_interface(ifaces: List[dict], default_iface: Optional[str]) -> Optional[dict]:
    """The interface students reach the Deck on: the default-route interface if it can do multicast,
    otherwise the first real (non-virtual) multicast interface. None = let CycloneDDS decide."""
    def usable(i):
        ip = ipaddress.ip_address(i["ip"]) if i.get("ip") else None
        return (ip is not None and not ip.is_loopback and not ip.is_link_local and "UP" in i["flags"]
                and "MULTICAST" in i["flags"] and "LOOPBACK" not in i["flags"] and "POINTOPOINT" not in i["flags"]
                and i.get("state") in ("UP", "UNKNOWN"))
    cands = [i for i in ifaces if usable(i)]
    for i in cands:
        if i["name"] == default_iface and not i["name"].startswith(VIRTUAL_PREFIXES):
            return i
    real = [i for i in cands if not i["name"].startswith(VIRTUAL_PREFIXES)]
    return (real or [None])[0]


def resolve_student_interface(setting: str = "auto") -> Tuple[Optional[str], str]:
    """(interface name for the CycloneDDS config or None, human description like 'wlan0 192.168.88.250')."""
    ifaces = list_ipv4_interfaces()
    if setting and setting != "auto":
        ips = [i["ip"] for i in ifaces if i["name"] == setting]
        return setting, f"{setting} {ips[0] if ips else '(no IPv4 address!)'}"
    best = choose_student_interface(ifaces, default_route_interface())
    if best is None:
        return None, "chosen by CycloneDDS (no usable network interface found)"
    return best["name"], f"{best['name']} {best['ip']}"


def cyclone_xml(private_domains, student_domain: Optional[int], student_interface: Optional[str] = "auto") -> str:
    parts = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<CycloneDDS xmlns="https://cdds.io/config">']
    if student_domain is not None:
        if student_interface == "auto":
            student_interface = resolve_student_interface("auto")[0]
        ifc = ""
        if student_interface:
            ifc = f'<Interfaces><NetworkInterface name="{student_interface}"/></Interfaces>'
        parts.append(NETWORK_DOMAIN.format(domain=student_domain, interfaces=ifc))
    for d in private_domains:
        parts.append(LOOPBACK_DOMAIN.format(domain=d))
    parts.append("</CycloneDDS>")
    return "\n".join(parts) + "\n"


def write_cyclone_config(name: str, private_domains, student_domain: Optional[int],
                         student_interface: str = "auto") -> str:
    os.makedirs(RUN_DIR, exist_ok=True)
    path = os.path.join(RUN_DIR, f"cyclonedds-{name}.xml")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(cyclone_xml(private_domains, student_domain, student_interface))
    os.replace(tmp, path)
    return path


def ros_env(domain: int, cyclone_path: str, base: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    env = dict(base if base is not None else os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["CYCLONEDDS_URI"] = "file://" + cyclone_path
    # ROS_LOCALHOST_ONLY would override our per-domain interface choice
    env.pop("ROS_LOCALHOST_ONLY", None)
    return env
