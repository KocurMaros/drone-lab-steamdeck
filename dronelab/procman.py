"""Starting/stopping helper processes from the GUIs."""
from __future__ import annotations

import glob
import os
import re
import signal
import socket
import subprocess
import sys
from typing import Dict, List, Optional

from .config import REPO_ROOT, RUN_DIR, VAR_DIR


def free_udp_port(start: int, end: int = 65000) -> int:
    for port in range(start, end):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", port))
            return port
        except OSError:
            continue
        finally:
            s.close()
    raise RuntimeError("no free UDP port")


def log_path(name: str) -> str:
    d = os.path.join(VAR_DIR, "logs")
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, name)


def spawn(args: List[str], log_name: str, env: Optional[Dict[str, str]] = None, cwd: str = REPO_ROOT,
          new_session: bool = True) -> subprocess.Popen:
    """Start a long-running helper with output in var/logs/<log_name>."""
    out = open(log_path(log_name), "w", encoding="utf-8")
    e = dict(os.environ if env is None else env)
    e["PYTHONPATH"] = REPO_ROOT + (os.pathsep + e["PYTHONPATH"] if e.get("PYTHONPATH") else "")
    e["PYTHONUNBUFFERED"] = "1"
    return subprocess.Popen(args, cwd=cwd, env=e, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=new_session)


def spawn_gate(sysid: int, fcu_url: str, profile: Optional[str] = None, student_domain: Optional[int] = None,
               tgt_component: int = 1) -> subprocess.Popen:
    args = [sys.executable, "-m", "dronelab.gate", "--sysid", str(sysid), "--fcu-url", fcu_url,
            "--tgt-component", str(tgt_component)]
    if profile:
        args += ["--profile", profile]
    if student_domain is not None:
        args += ["--student-domain", str(student_domain)]
    return spawn(args, f"gate-{sysid}.log")


def existing_gate_sockets() -> Dict[int, str]:
    out = {}
    for p in glob.glob(os.path.join(RUN_DIR, "gate-*.sock")):
        m = re.search(r"gate-(\d+)\.sock$", p)
        if m:
            out[int(m.group(1))] = p
    return out


def kill_group(proc: subprocess.Popen, timeout: float = 5.0):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
        try:
            os.killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError):
            return
        try:
            proc.wait(wait)
            return
        except subprocess.TimeoutExpired:
            continue


def tail(path: str, n: int = 40) -> str:
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 16000))
            return "\n".join(f.read().decode(errors="replace").splitlines()[-n:])
    except OSError:
        return ""
