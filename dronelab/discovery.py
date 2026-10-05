"""Find MAVLink vehicles on whatever subnet / port they happen to use.

Two mechanisms run side by side:
  * listen: bind UDP ports the drones' mavlink-router pushes to ("Normal" endpoints)
  * probe:  send a GCS heartbeat to candidate IPs on the router's "Server" port; the
            router answers to whoever talked to it last.

A found vehicle is reported with a ready-to-use MAVROS fcu_url. Before MAVROS
is started for a path, call `release(path)`: it closes our listener on that
port / stops probing that IP, because a mavlink-router Server endpoint only
talks to ONE client at a time and a listen port can only be bound once.
"""
from __future__ import annotations

import errno
import ipaddress
import json
import selectors
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

from . import mavlink_lite as ml


@dataclass
class Path:
    via: str            # "listen" or "probe"
    ip: str             # drone IP
    port: int           # listen: our local port; probe: drone's server port

    @property
    def key(self) -> str:
        return f"{self.via}:{self.ip}:{self.port}"

    def fcu_url(self, local_port: Optional[int] = None) -> str:
        if self.via == "listen":
            # bind our port, learn the drone's address from its first packet
            return f"udp://0.0.0.0:{self.port}@"
        lp = local_port if local_port is not None else 0
        return f"udp://0.0.0.0:{lp}@{self.ip}:{self.port}"

    def describe(self) -> str:
        return f"{self.ip} -> :{self.port}" if self.via == "listen" else f"{self.ip}:{self.port} (server)"


@dataclass
class Vehicle:
    sysid: int
    compid: int
    hb: ml.Heartbeat
    paths: Dict[str, Tuple[Path, float]] = field(default_factory=dict)  # key -> (path, last_seen)
    first_seen: float = 0.0
    hb_count: int = 0

    def best_path(self, now: float, stale: float) -> Optional[Path]:
        fresh = [(p, t) for p, t in self.paths.values() if now - t <= stale]
        if not fresh:
            return None
        # prefer pushed streams (no single-client limitation), then most recent
        fresh.sort(key=lambda pt: (pt[0].via != "listen", -pt[1]))
        return fresh[0][0]

    def last_seen(self) -> float:
        return max((t for _, t in self.paths.values()), default=0.0)


def local_ipv4_networks() -> List[ipaddress.IPv4Interface]:
    """IPv4 addresses of this machine (excluding loopback), best effort without extra deps."""
    out: List[ipaddress.IPv4Interface] = []
    try:
        import psutil  # type: ignore
        for name, addrs in psutil.net_if_addrs().items():
            for a in addrs:
                if a.family == socket.AF_INET and a.netmask and not a.address.startswith("127."):
                    out.append(ipaddress.IPv4Interface(f"{a.address}/{a.netmask}"))
        return out
    except Exception:
        pass
    try:
        js = subprocess.run(["ip", "-j", "-4", "addr", "show"], capture_output=True, text=True, timeout=2).stdout
        for ifc in json.loads(js or "[]"):
            for a in ifc.get("addr_info", []):
                if a.get("family") == "inet" and not a["local"].startswith("127."):
                    out.append(ipaddress.IPv4Interface(f"{a['local']}/{a['prefixlen']}"))
    except Exception:
        pass
    return out


def subnet_hosts_24(ifaces: Iterable[ipaddress.IPv4Interface]) -> List[str]:
    """.1 - .254 of the /24 around each local address (wider subnets are clipped to /24)."""
    hosts: List[str] = []
    own = set()
    for itf in ifaces:
        own.add(str(itf.ip))
        net = ipaddress.ip_network(f"{itf.ip}/24", strict=False)
        hosts.extend(str(h) for h in net.hosts())
    return [h for h in dict.fromkeys(hosts) if h not in own]


class Discovery:
    def __init__(self, listen_ports: List[int], probe_targets: List[str], probe_port: int = 14550,
                 probe_local_subnets: bool = True, probe_interval: float = 3.0, stale_after: float = 4.0,
                 log: Callable[[str], None] = print):
        self.listen_ports = list(listen_ports)
        self.probe_targets = list(probe_targets)
        self.probe_port = probe_port
        self.probe_local_subnets = probe_local_subnets
        self.probe_interval = probe_interval
        self.stale_after = stale_after
        self.log = log
        self._sel = selectors.DefaultSelector()
        self._listeners: Dict[int, socket.socket] = {}
        self._probe_sock: Optional[socket.socket] = None
        self._vehicles: Dict[int, Vehicle] = {}
        self._released_ports: Set[int] = set()
        self._released_ips: Set[str] = set()
        self._busy_ports: Set[int] = set()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._seq = 0
        self.local_ifaces: List[ipaddress.IPv4Interface] = []
        self.errors: List[str] = []

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        self._open_sockets()
        self._thread = threading.Thread(target=self._run, name="mavlink-discovery", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        for s in list(self._listeners.values()):
            self._close(s)
        self._listeners.clear()
        if self._probe_sock:
            self._close(self._probe_sock)
            self._probe_sock = None

    def _close(self, s: socket.socket):
        try:
            self._sel.unregister(s)
        except Exception:
            pass
        try:
            s.close()
        except Exception:
            pass

    def _open_sockets(self):
        busy = []
        for port in self.listen_ports:
            if port in self._released_ports or port in self._listeners:
                continue
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.bind(("0.0.0.0", port))
            except OSError as e:
                s.close()
                if e.errno == errno.EADDRINUSE:
                    busy.append(port)
                    continue
                raise
            s.setblocking(False)
            self._listeners[port] = s
            self._sel.register(s, selectors.EVENT_READ, ("listen", port))
        with self._lock:
            self._busy_ports = set(busy)
        if busy:
            self.log(f"[discovery] ports already in use (MAVROS or another app?): {self._compress(busy)}")
        if self._probe_sock is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.bind(("0.0.0.0", 0))
            s.setblocking(False)
            self._probe_sock = s
            self._sel.register(s, selectors.EVENT_READ, ("probe", 0))

    @staticmethod
    def _compress(ports: List[int]) -> str:
        ports = sorted(ports)
        out, start, prev = [], None, None
        for p in ports + [None]:
            if start is None:
                start = prev = p
            elif p is not None and p == prev + 1:
                prev = p
            else:
                out.append(f"{start}" if start == prev else f"{start}-{prev}")
                start = prev = p
        return ", ".join(out)

    # ------------------------------------------------------------------ control
    def release(self, path: Path):
        """Free a path before MAVROS uses it."""
        with self._lock:
            self._released_ips.add(path.ip)
            if path.via == "listen":
                self._released_ports.add(path.port)
        if path.via == "listen" and path.port in self._listeners:
            s = self._listeners.pop(path.port)
            self._close(s)

    def reclaim(self, path: Path):
        """MAVROS for this path stopped: resume listening / probing."""
        with self._lock:
            self._released_ips.discard(path.ip)
            self._released_ports.discard(path.port)
        try:
            self._open_sockets()
        except OSError as e:
            self.log(f"[discovery] could not reopen port {path.port}: {e}")

    def forget(self, sysid: int):
        with self._lock:
            self._vehicles.pop(sysid, None)

    def snapshot(self) -> List[dict]:
        now = time.monotonic()
        rows = []
        with self._lock:
            for v in sorted(self._vehicles.values(), key=lambda v: v.sysid):
                best = v.best_path(now, self.stale_after)
                age = now - v.last_seen()
                rows.append(dict(
                    sysid=v.sysid, compid=v.compid, mode=v.hb.mode_name, armed=v.hb.armed,
                    type=v.hb.type_name, autopilot=v.hb.autopilot_name, age=age,
                    stale=age > self.stale_after,
                    path=best, paths=[p for p, t in v.paths.values() if now - t <= self.stale_after],
                ))
        return rows

    @property
    def busy_ports(self) -> Set[int]:
        with self._lock:
            return set(self._busy_ports)

    # ------------------------------------------------------------------ worker
    def _probe_list(self) -> List[str]:
        targets = list(self.probe_targets)
        if self.probe_local_subnets:
            self.local_ifaces = local_ipv4_networks()
            targets += subnet_hosts_24(self.local_ifaces)
        with self._lock:
            skip = set(self._released_ips)
        return [t for t in dict.fromkeys(targets) if t not in skip]

    def _send_probes(self):
        if not self._probe_sock:
            return
        self._seq = (self._seq + 1) & 0xFF
        pkt = ml.pack_heartbeat(seq=self._seq)
        for ip in self._probe_list():
            try:
                self._probe_sock.sendto(pkt, (ip, self.probe_port))
            except OSError:
                pass  # unreachable network etc.

    def _run(self):
        next_probe = 0.0
        next_reopen = time.monotonic() + 5.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_probe:
                self._send_probes()
                next_probe = now + self.probe_interval
            if now >= next_reopen:
                # retry ports that were busy (e.g. an old MAVROS just exited)
                try:
                    self._open_sockets()
                except OSError:
                    pass
                next_reopen = now + 5.0
            for key, _ in self._sel.select(timeout=0.2):
                sock = key.fileobj
                via, port = key.data
                for _ in range(64):  # drain
                    try:
                        data, (ip, rport) = sock.recvfrom(4096)
                    except (BlockingIOError, InterruptedError):
                        break
                    except OSError:
                        break
                    if via == "probe":
                        with self._lock:
                            if ip in self._released_ips:
                                continue
                    self._ingest(data, Path(via, ip, port if via == "listen" else rport))

    def _ingest(self, data: bytes, path: Path):
        for hb in ml.heartbeats(data):
            if not hb.is_vehicle:
                continue
            now = time.monotonic()
            with self._lock:
                v = self._vehicles.get(hb.sysid)
                if v is None:
                    v = Vehicle(hb.sysid, hb.compid, hb, first_seen=now)
                    self._vehicles[hb.sysid] = v
                    self.log(f"[discovery] found system {hb.sysid} ({hb.autopilot_name} {hb.type_name}) "
                             f"via {path.via} {path.describe()}")
                v.hb = hb
                v.hb_count += 1
                v.paths[path.key] = (path, now)
