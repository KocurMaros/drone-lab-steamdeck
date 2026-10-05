"""ROS 2 / DDS environment for the two-domain isolation.

MAVROS runs on a private domain that CycloneDDS binds to the loopback interface
only; students on the lab network cannot discover or talk to it. The gate
process is a member of both domains, so it gets a CycloneDDS configuration with
one <Domain> section per domain.
"""
from __future__ import annotations

import os
from typing import Dict, Optional

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

NETWORK_DOMAIN = """
  <Domain Id="{domain}">
    <General>{interfaces}</General>
  </Domain>"""


def cyclone_xml(private_domains, student_domain: Optional[int], student_interface: str = "auto") -> str:
    parts = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<CycloneDDS xmlns="https://cdds.io/config">']
    if student_domain is not None:
        ifc = ""
        if student_interface and student_interface != "auto":
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
