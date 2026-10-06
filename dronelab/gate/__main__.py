"""Run MAVROS + the student gate for one drone.

    python3 -m dronelab.gate --sysid 11 --fcu-url udp://0.0.0.0:14511@ [--profile indoor_lab]

Normally started by the Flight / Demo apps; can be run by hand in a terminal.
"""
from __future__ import annotations

import argparse
import collections
import ctypes
import os
import signal
import subprocess
import sys
import threading

from ..config import VAR_DIR, Config
from ..dds import resolve_student_interface, ros_env, write_cyclone_config
from .admin import AdminServer


def _pdeathsig():
    """Child dies with SIGTERM when the gate dies (even on SIGKILL), so MAVROS never outlives it."""
    try:
        ctypes.CDLL("libc.so.6").prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except Exception:
        pass
    os.setsid()


class Mavros:
    def __init__(self, ds, fcu_url: str, tgt_component: int, log):
        self.ds, self.fcu_url, self.tgt_component, self.log = ds, fcu_url, tgt_component, log
        self.proc = None
        self.tail = collections.deque(maxlen=200)
        os.makedirs(os.path.join(VAR_DIR, "logs"), exist_ok=True)
        self.logfile = os.path.join(VAR_DIR, "logs", f"mavros-{ds.sysid}.log")

    def start(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        xml = write_cyclone_config(f"mavros-{self.ds.sysid}", [self.ds.private_domain], None)
        env = ros_env(self.ds.private_domain, xml)
        # Same parameters as `ros2 launch mavros apm.launch`, plus system_id: MAVROS identifies
        # itself as the GCS (SYSID_MYGCS, 255 by default) so ArduPilot's GCS failsafe can work.
        share = os.path.join(get_package_share_directory("mavros"), "launch")
        flavour = self.ds.mavros_launch
        # exec the node binary directly (not `ros2 run`, which wraps it in another process) so the
        # parent-death signal and the process-group kill reach MAVROS itself
        binary = os.path.join(get_package_prefix("mavros"), "lib", "mavros", "mavros_node")
        cmd = [binary, "--ros-args",
               "--params-file", os.path.join(share, f"{flavour}_pluginlists.yaml"),
               "--params-file", os.path.join(share, f"{flavour}_config.yaml"),
               "-p", f"fcu_url:={self.fcu_url}",
               "-p", f"tgt_system:={self.ds.sysid}", "-p", f"tgt_component:={self.tgt_component}",
               "-p", "fcu_protocol:=v2.0", "-p", f"system_id:={self.ds.mavros_system_id}"]
        if self.ds.gcs_url:   # an empty "-p gcs_url:=" is a parse error in rcl
            cmd += ["-p", f"gcs_url:={self.ds.gcs_url}"]
        self.log(f"[gate] starting MAVROS: {' '.join(cmd)}  (ROS_DOMAIN_ID={self.ds.private_domain}, loopback)")
        self.proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, preexec_fn=_pdeathsig, text=True, bufsize=1,
                                     errors="replace")
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        # must keep draining stdout whatever happens, or MAVROS blocks on a full pipe
        try:
            f = open(self.logfile, "w", encoding="utf-8")
        except OSError:
            f = None
        for line in self.proc.stdout:
            self.tail.append(line.rstrip())
            if f is not None:
                try:
                    f.write(line)
                    f.flush()
                except OSError:
                    f = None

    @property
    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if not self.proc or self.proc.poll() is not None:
            return
        for sig, wait in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(wait)
                return
            except subprocess.TimeoutExpired:
                continue


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sysid", type=int, required=True)
    ap.add_argument("--fcu-url", required=True)
    ap.add_argument("--tgt-component", type=int, default=1)
    ap.add_argument("--profile", default=None, help="fence profile name from the config")
    ap.add_argument("--config", default=None)
    ap.add_argument("--student-domain", type=int, default=None, help="override network.student_domain")
    ap.add_argument("--mavros-ns", default="/mavros")
    a = ap.parse_args(argv)

    def log(msg):
        print(msg, flush=True)

    cfg = Config.load(a.config)
    ds = cfg.drone(a.sysid, a.profile)
    if a.student_domain is not None:
        ds.student_domain = a.student_domain
        if ds.student_domain == ds.private_domain:
            sys.exit("student domain must differ from the private domain")

    if AdminServer.already_running(ds.socket_path):
        sys.exit(f"a gate for system {a.sysid} is already running ({ds.socket_path})")

    # DDS environment must be in place before rclpy is imported
    iface, iface_desc = resolve_student_interface(ds.student_interface)
    xml = write_cyclone_config(f"gate-{ds.sysid}", [ds.private_domain], ds.student_domain, iface)
    os.environ.update(ros_env(ds.private_domain, xml))
    log(f"[gate] students reach drone {ds.sysid} on ROS_DOMAIN_ID={ds.student_domain} via {iface_desc}")

    mav = Mavros(ds, a.fcu_url, a.tgt_component, log)
    mav.start()

    from .node import Gate  # noqa: E402  (imports rclpy)
    gate = Gate(ds, a.mavros_ns, log=log)

    def status():
        st = gate.status()
        st.update(mavros_alive=mav.alive, fcu_url=a.fcu_url, pid=os.getpid(), student_iface=iface_desc,
                  mavros_tail=list(mav.tail)[-6:])
        return st

    stop = threading.Event()

    def command(cmd):
        if cmd.get("cmd") == "shutdown":
            st = gate.status()
            if st.get("armed") and st.get("in_air") and not cmd.get("force"):
                return {"ok": False, "msg": "drone is flying: shutdown refused (send force=true to override)"}
            stop.set()
            return {"ok": True, "msg": "shutting down"}
        if cmd.get("cmd") == "mavros_log":
            return {"ok": True, "msg": "\n".join(mav.tail)}
        return gate.admin(cmd)

    admin = AdminServer(ds.socket_path, status, command)
    admin.start()
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, lambda *_: stop.set())

    warned = set()
    while not stop.wait(0.5):
        if not mav.alive and "mavros" not in warned:
            gate._error("MAVROS exited - see the MAVROS log in the Flight app", key="mavros_dead", force=True)
            warned.add("mavros")
        if not gate.executors_alive and "executor" not in warned:
            # the gate can no longer validate or relay: get the drone down and tell the instructor
            warned.add("executor")
            gate._error("INTERNAL GATE FAILURE (executor died) - landing; restart the gate", key="exec_dead",
                        force=True)
            try:
                gate.cg.lock("internal gate failure")
                gate._enforce_mode("LAND", "internal gate failure")
            except Exception as e:
                log(f"[gate] could not command LAND: {e}")
    log("[gate] shutting down")
    admin.stop()
    mav.stop()
    gate.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
