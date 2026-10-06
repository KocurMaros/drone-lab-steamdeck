"""Start/stop the whole simulation: Gazebo (server + optional chase-cam client),
ArduCopter SITL and the student gate (MAVROS on a private domain)."""
from __future__ import annotations

import os
import shutil
import subprocess
import threading
import time
from typing import Callable, Dict, Optional, Tuple

from .. import procman
from ..config import REPO_ROOT, VAR_DIR, Config
from ..dds import ros_env, write_cyclone_config

MODEL = "dronelab_iris"


def _exp(p: str) -> str:
    return os.path.abspath(os.path.expanduser(os.path.expandvars(p)))


class SimStack:
    def __init__(self, cfg: Config, log: Callable[[str], None] = print):
        self.cfg = cfg
        d = cfg.data.get("demo", {}) or {}
        self.d = d
        self.log = log
        self.student_domain = int(d.get("student_domain", cfg.student_domain))
        self.ardupilot = _exp(d.get("ardupilot_dir", "~/ardupilot"))
        self.world = os.path.join(REPO_ROOT, d.get("world", "sim/worlds/dronelab_arena.world"))
        self.home = d.get("home", "48.1508457,17.0727297,140,0")
        self.procs: Dict[str, subprocess.Popen] = {}
        self.state: Dict[str, str] = {"gazebo": "stopped", "sitl": "stopped", "gate": "stopped",
                                      "gazebo_view": "off"}
        self._stop = threading.Event()
        self.gazebo_geometry: Optional[Tuple[int, int, int, int]] = None

    # ------------------------------------------------------------------ env
    def gazebo_env(self) -> Dict[str, str]:
        paths = [os.path.join(REPO_ROOT, "sim", "models"), _exp(self.d.get("ardupilot_gazebo_models",
                                                                            "~/ardupilot_gazebo/models")),
                 "/usr/share/gazebo-11/models"]
        xml = write_cyclone_config("demo", [], self.student_domain)
        env = ros_env(self.student_domain, xml)
        env["GAZEBO_MODEL_PATH"] = ":".join(p for p in paths if os.path.isdir(p)) + \
            (":" + env["GAZEBO_MODEL_PATH"] if env.get("GAZEBO_MODEL_PATH") else "")
        env["GAZEBO_MODEL_DATABASE_URI"] = ""   # never hang trying to download models
        # server and client run on this machine: talk over loopback, so a Wi-Fi change (new IP) during a
        # demo cannot cut the Gazebo window off
        env["GAZEBO_IP"] = "127.0.0.1"
        env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
        if os.path.isfile("/usr/share/gazebo/setup.sh"):
            env.setdefault("GAZEBO_RESOURCE_PATH", "/usr/share/gazebo-11")
            env.setdefault("GAZEBO_PLUGIN_PATH", "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins")
        return env

    def _gz(self, *args, timeout=5.0) -> Optional[str]:
        try:
            r = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout, env=self.gazebo_env())
            return r.stdout if r.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired):
            return None

    # ------------------------------------------------------------------ lifecycle
    def start(self, show_gazebo: bool = True):
        threading.Thread(target=self._start, args=(show_gazebo,), daemon=True, name="sim-start").start()

    def _alive(self, name: str) -> bool:
        p = self.procs.get(name)
        return p is not None and p.poll() is None

    def _start(self, show_gazebo: bool):
        self._stop.clear()
        # 1) Gazebo server
        if not self._alive("gzserver"):
            if not os.path.isfile(self.world):
                self.state["gazebo"] = f"world missing: {self.world}"
                return
            self.state["gazebo"] = "starting"
            self.procs["gzserver"] = procman.spawn(
                ["gzserver", "--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so",
                 self.world], "gzserver.log", env=self.gazebo_env())
            t0 = time.monotonic()
            while time.monotonic() - t0 < 90 and not self._stop.is_set():
                if not self._alive("gzserver"):
                    self.state["gazebo"] = "CRASHED (see logs/gzserver.log)"
                    return
                if self._gz("model", "-m", MODEL, "-p"):
                    break
                time.sleep(1.0)
            else:
                self.state["gazebo"] = "timeout"
                return
        self.state["gazebo"] = "running"
        if show_gazebo:
            self.start_view()
        # 2) ArduCopter SITL
        if not self._alive("sitl"):
            binary = os.path.join(self.ardupilot, "build", "sitl", "bin", "arducopter")
            if not os.path.isfile(binary):
                self.state["sitl"] = f"not built: {binary}"
                return
            dp = os.path.join(self.ardupilot, "Tools", "autotest", "default_params")
            defaults = ",".join([os.path.join(dp, "copter.parm"), os.path.join(dp, "gazebo-iris.parm"),
                                 os.path.join(REPO_ROOT, "sim", "params", "demo.parm")])
            wd = os.path.join(VAR_DIR, "sitl")
            os.makedirs(wd, exist_ok=True)
            self.state["sitl"] = "starting"
            self.procs["sitl"] = procman.spawn(
                [binary, "-S", "--model", "gazebo-iris", "--speedup", "1", "-I0", "-w", "--home", self.home,
                 "--defaults", defaults], "sitl.log", cwd=wd)
        time.sleep(2.0)
        self.state["sitl"] = "running" if self._alive("sitl") else "CRASHED (see logs/sitl.log)"
        if not self._alive("sitl"):
            return
        # 3) gate + MAVROS (SITL's SERIAL0 is TCP 5760)
        if not self._alive("gate"):
            self.state["gate"] = "starting"
            self.procs["gate"] = procman.spawn_gate(1, "tcp://127.0.0.1:5760", "sim_arena", self.student_domain)
        self.state["gate"] = "running"

    def start_view(self):
        if self._alive("gzclient"):
            return
        self.state["gazebo_view"] = "starting"
        self.procs["gzclient"] = procman.spawn(["gzclient", "--verbose"], "gzclient.log", env=self.gazebo_env())
        threading.Thread(target=self._place_view, daemon=True).start()

    def _place_view(self):
        """Move the Gazebo window to the second screen (needs xdotool; wmctrl optional)."""
        if not self.gazebo_geometry:
            self.state["gazebo_view"] = "running"
            return
        if not shutil.which("xdotool"):
            self.state["gazebo_view"] = "running (install xdotool to auto-place it)"
            return
        x, y, w, h = self.gazebo_geometry
        for _ in range(60):
            if not self._alive("gzclient"):
                return
            out = subprocess.run(["xdotool", "search", "--name", "^Gazebo$"], capture_output=True, text=True).stdout
            wins = out.split()
            if wins:
                wid = wins[-1]
                subprocess.run(["xdotool", "windowmove", wid, str(x), str(y)])
                subprocess.run(["xdotool", "windowsize", wid, str(w), str(h)])
                if shutil.which("wmctrl"):
                    subprocess.run(["wmctrl", "-i", "-r", wid, "-b", "add,fullscreen"])
                self.state["gazebo_view"] = "running"
                return
            time.sleep(1.0)
        self.state["gazebo_view"] = "running"

    def stop(self):
        self._stop.set()
        for name in ("gate", "sitl", "gzclient", "gzserver"):
            p = self.procs.pop(name, None)
            if p is not None:
                procman.kill_group(p, timeout=6.0 if name == "gate" else 3.0)
        for k in self.state:
            self.state[k] = "stopped" if k != "gazebo_view" else "off"

    def check(self) -> Dict[str, str]:
        for name, key in (("gzserver", "gazebo"), ("sitl", "sitl"), ("gate", "gate")):
            p = self.procs.get(name)
            if p is not None and p.poll() is not None and not self.state[key].startswith("CRASHED"):
                self.state[key] = f"CRASHED (exit {p.returncode}, see logs/{'gate-1' if name == 'gate' else name}.log)"
        p = self.procs.get("gzclient")
        if p is not None and p.poll() is not None:
            self.state["gazebo_view"] = "closed"
        return dict(self.state)

    # ------------------------------------------------------------------ teleport
    def teleport_cli(self, x_enu: float, y_enu: float, z: float = 0.05) -> bool:
        """Fallback when the ROS set_entity_state service is unavailable."""
        gx, gy = y_enu, -x_enu
        return self._gz("model", "-m", MODEL, "-x", str(gx), "-y", str(gy), "-z", str(z),
                        "-R", "0", "-P", "0", "-Y", "0") is not None
