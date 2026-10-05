"""Private control channel between the gate and the Steam Deck GUIs.

A UNIX domain socket (JSON lines) inside the container: students on the network
cannot reach it, and a GUI that crashes or restarts can simply reconnect while
the gate keeps protecting the drone.
"""
from __future__ import annotations

import json
import os
import socket
import threading
import time
from typing import Callable, Dict, List, Optional


class AdminServer:
    def __init__(self, path: str, status_fn: Callable[[], dict], command_fn: Callable[[dict], dict],
                 rate_hz: float = 5.0):
        self.path = path
        self.status_fn = status_fn
        self.command_fn = command_fn
        self.period = 1.0 / rate_hz
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._clients: List[socket.socket] = []
        self._send_locks: Dict[int, threading.Lock] = {}
        self._clients_lock = threading.Lock()

    @staticmethod
    def already_running(path: str) -> bool:
        if not os.path.exists(path):
            return False
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(0.5)
        try:
            s.connect(path)
            return True
        except OSError:
            return False
        finally:
            s.close()

    def start(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        if os.path.exists(self.path):
            os.unlink(self.path)  # stale (already_running() was checked by the caller)
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.bind(self.path)
        os.chmod(self.path, 0o600)
        s.listen(8)
        s.settimeout(0.5)
        self._sock = s
        threading.Thread(target=self._accept_loop, name="admin-accept", daemon=True).start()

    def stop(self):
        self._stop.set()
        for c in list(self._clients):
            try:
                c.close()
            except OSError:
                pass
        if self._sock:
            self._sock.close()
        try:
            os.unlink(self.path)
        except OSError:
            pass

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self._clients_lock:
                self._clients.append(conn)
                self._send_locks[conn.fileno()] = threading.Lock()
            threading.Thread(target=self._client_rx, args=(conn,), daemon=True).start()
            threading.Thread(target=self._client_tx, args=(conn,), daemon=True).start()

    def _send(self, conn: socket.socket, obj: dict) -> bool:
        lk = self._send_locks.get(conn.fileno()) if conn.fileno() >= 0 else None
        try:
            data = (json.dumps(obj, default=str) + "\n").encode()
            if lk is None:
                return False
            with lk:
                conn.sendall(data)
            return True
        except (OSError, ValueError, TypeError):
            return False

    def _client_tx(self, conn: socket.socket):
        while not self._stop.is_set():
            try:
                st = self.status_fn()
            except Exception as e:  # never let a status bug kill the channel
                st = {"type": "status", "error": f"status failed: {e}"}
            st["type"] = "status"
            if not self._send(conn, st):
                break
            time.sleep(self.period)
        self._drop(conn)

    def _client_rx(self, conn: socket.socket):
        buf = b""
        while not self._stop.is_set():
            try:
                data = conn.recv(4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            if len(buf) > 1_000_000:   # garbage without newlines: drop the client
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    cmd = json.loads(line)
                except (ValueError, RecursionError):
                    cmd = None
                if not isinstance(cmd, dict):
                    self._send(conn, {"type": "reply", "ok": False, "msg": "expected a JSON object"})
                    continue
                # run commands off the rx thread so a slow MAVROS call never blocks reading
                threading.Thread(target=self._run_cmd, args=(conn, cmd), daemon=True).start()
        self._drop(conn)

    def _run_cmd(self, conn, cmd):
        try:
            res = self.command_fn(cmd)
        except Exception as e:
            res = {"ok": False, "msg": f"{type(e).__name__}: {e}"}
        res.update({"type": "reply", "id": cmd.get("id"), "cmd": cmd.get("cmd")})
        self._send(conn, res)

    def _drop(self, conn):
        with self._clients_lock:
            if conn in self._clients:
                self._clients.remove(conn)
        try:
            conn.close()
        except OSError:
            pass


class AdminClient:
    """Used by the GUIs. Thread-safe; call poll() from the GUI timer."""

    def __init__(self, path: str):
        self.path = path
        self.sock: Optional[socket.socket] = None
        self.buf = b""
        self.last_status: Dict = {}
        self.last_status_time = 0.0
        self.replies: List[dict] = []
        self._next_id = 1
        self._lock = threading.Lock()

    def connect(self) -> bool:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(1.0)
        try:
            s.connect(self.path)
        except OSError:
            s.close()
            return False
        s.setblocking(False)
        self.sock = s
        return True

    @property
    def connected(self) -> bool:
        return self.sock is not None

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def send(self, cmd: str, **kw) -> Optional[int]:
        if not self.sock and not self.connect():
            return None
        with self._lock:
            cid = self._next_id
            self._next_id += 1
        msg = dict(kw, cmd=cmd, id=cid)
        try:
            self.sock.setblocking(True)
            self.sock.sendall((json.dumps(msg) + "\n").encode())
            self.sock.setblocking(False)
        except OSError:
            self.close()
            return None
        return cid

    def poll(self) -> Optional[dict]:
        """Read everything available; returns the newest status (or None)."""
        if not self.sock and not self.connect():
            return None
        newest = None
        while True:
            try:
                data = self.sock.recv(65536)
            except BlockingIOError:
                break
            except OSError:
                self.close()
                break
            if not data:
                self.close()
                break
            self.buf += data
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if obj.get("type") == "status":
                newest = obj
                self.last_status = obj
                self.last_status_time = time.monotonic()
            else:
                self.replies.append(obj)
        return newest

    def pop_replies(self) -> List[dict]:
        r, self.replies = self.replies, []
        return r
