"""DroneLab Flight: connect real drones through the safety gate (Steam Deck touch UI).

    python3 -m dronelab.gui.flight_app [--windowed]
"""
from __future__ import annotations

import argparse
import collections
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QComboBox, QDialog, QFormLayout, QFrame, QHBoxLayout,
                             QLineEdit, QListWidget, QListWidgetItem, QMainWindow, QPlainTextEdit, QScrollArea,
                             QSpinBox, QStackedWidget, QVBoxLayout, QWidget)

from .. import procman
from ..config import VAR_DIR, Config
from ..discovery import Discovery, Path
from ..gate.admin import AdminClient
from . import theme
from .widgets import Confirm, FenceMap, HoldButton, Tile, button, card, label

LANDED = {0: "?", 1: "on ground", 2: "in air", 3: "taking off", 4: "landing"}


# ============================================================================ connected drone panel
class GateHandle:
    def __init__(self, sysid: int, sock: str, proc=None, path: Optional[Path] = None):
        self.sysid = sysid
        self.client = AdminClient(sock)
        self.proc = proc
        self.path = path
        self.started = time.monotonic()
        self.status: dict = {}
        self.square: Optional[subprocess.Popen] = None     # the square demo, if running
        self.square_lines: collections.deque = collections.deque(maxlen=200)

    @property
    def square_running(self) -> bool:
        return self.square is not None and self.square.poll() is None

    @property
    def alive(self) -> bool:
        if self.proc is not None:
            return self.proc.poll() is None
        return self.client.connected or os.path.exists(self.client.path)


class DronePanel(QWidget):
    def __init__(self, app: "FlightWindow", h: GateHandle):
        super().__init__()
        self.app, self.h = app, h
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        self.banner = QFrame()
        self.banner.setObjectName("banner")
        bl = QHBoxLayout(self.banner)
        self.banner_text = label("", wrap=True)
        bl.addWidget(self.banner_text, 1)
        self.banner.hide()
        root.addWidget(self.banner)

        tiles = QHBoxLayout()
        self.t_mode, self.t_arm, self.t_batt = Tile("MODE"), Tile("STATE"), Tile("BATTERY")
        self.t_link, self.t_pose, self.t_fence = Tile("LINK"), Tile("POSITION"), Tile("FENCE")
        for t in (self.t_mode, self.t_arm, self.t_batt, self.t_link, self.t_pose, self.t_fence):
            tiles.addWidget(t)
        root.addLayout(tiles)

        mid = QHBoxLayout()
        self.map = FenceMap()
        mid.addWidget(self.map, 3)
        info = card()
        il = QVBoxLayout(info)
        il.setContentsMargins(14, 12, 14, 12)
        self.i_title = label("", "h2")
        self.i_ns = label("", "mono", wrap=True)
        self.i_ns.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.i_activity = label("", "mono", wrap=True)
        self.i_stats = label("", "muted", wrap=True)
        for w in (self.i_title, self.i_ns, label("Student commands (age):", "muted"), self.i_activity, self.i_stats):
            il.addWidget(w)
        il.addStretch(1)
        mid.addWidget(info, 2)
        root.addLayout(mid, 1)

        ctl = QHBoxLayout()
        self.b_land = button("LAND", "danger", 60)
        self.b_brake = button("BRAKE", "warn", 60)
        self.b_loiter = button("LOITER", min_h=60)
        self.b_release = button("RELEASE LOCK", "warn", 60)
        self.b_kill = HoldButton("KILL (hold)", hold_s=2.0)
        self.b_kill.setMinimumHeight(60)
        self.b_more = button("···", min_h=60)
        self.b_more.setMaximumWidth(80)
        self.b_land.clicked.connect(lambda: self.app.admin(self.h, "land"))
        self.b_land.setToolTip("LAND now and lock student commands until RELEASE LOCK")
        self.b_brake.clicked.connect(lambda: self.app.admin(self.h, "brake"))
        self.b_loiter.clicked.connect(lambda: self.app.admin(self.h, "mode", mode="LOITER"))
        self.b_release.clicked.connect(self._release)
        self.b_kill.held.connect(lambda: self.app.admin(self.h, "kill"))
        self.b_more.clicked.connect(self._more)
        for b in (self.b_land, self.b_brake, self.b_loiter, self.b_release, self.b_kill, self.b_more):
            ctl.addWidget(b)
        root.addLayout(ctl)

        self.errors = QListWidget()
        self.errors.setMaximumHeight(130)
        root.addWidget(self.errors)
        self._last_err_key = None
        self._low_rate_since: Optional[float] = None

    def _release(self):
        st = self.h.status
        if Confirm.ask(self, "Release fence lock?",
                       f"Lock reason: {st.get('lock_reason') or '-'}\n\nStudents will be able to command the drone "
                       f"again. Make sure it is back inside the fence and stable.", "Release", danger=False):
            self.app.admin(self.h, "release_lock")

    def _more(self):
        d = QDialog(self)
        d.setWindowTitle(f"Drone {self.h.sysid}")
        d.setMinimumWidth(560)
        lay = QVBoxLayout(d)
        lay.setContentsMargins(20, 20, 20, 20)
        lay.addWidget(label(f"Drone {self.h.sysid}", "h2"))
        st = self.h.status
        if self.h.square_running:
            sq = ("STOP square demo (lands)", lambda: (d.accept(), self.app.stop_square(self.h)))
        else:
            sq = ("Fly 3 x 3 m square demo...", lambda: (d.accept(), self.app.fly_square(self.h)))
        n_warn = len(st.get("param_warnings") or [])
        items = [sq,
                 ("GUIDED - give control back to students",
                  lambda: (d.accept(), self.app.admin(self.h, "mode", mode="GUIDED"))),
                 (f"Drone parameters (RC takeover, failsafes){f'  -  {n_warn} warnings' if n_warn else ''}...",
                  lambda: (d.accept(), self.app.params_dialog(self.h))),
                 ("Show MAVROS / gate log", lambda: (d.accept(), self.app.show_log(self.h))),
                 ("RTL (return to launch)", lambda: (d.accept(), self.app.admin(self.h, "mode", mode="RTL"))),
                 ("Disarm (on the ground only)", lambda: (d.accept(), self.app.admin(self.h, "disarm"))),
                 ("Disconnect (stop gate + MAVROS)", lambda: (d.accept(), self.app.disconnect(self.h))),
                 ("Close", d.reject)]
        for text, fn in items:
            b = button(text, min_h=56)
            b.clicked.connect(fn)
            lay.addWidget(b)
        d.exec_()

    def refresh(self, st: dict, link_ok: bool):
        if not st:
            self.t_link.set("starting…" if link_ok else "NO GATE", theme.WARN if link_ok else theme.BAD)
            return
        mode = st.get("mode") or "-"
        pilot = st.get("pilot_control")
        if pilot:
            self.t_mode.set(f"{mode} (RC)", theme.WARN)
        else:
            self.t_mode.set(mode, theme.ACCENT if mode == "GUIDED" else theme.TEXT)
        armed = st.get("armed")
        ls = LANDED.get(st.get("landed_state", 0), "?")
        self.t_arm.set(("ARMED · " if armed else "disarmed · ") + ls, theme.WARN if armed else theme.TEXT)
        b = st.get("battery") or {}
        if b.get("voltage"):
            pct = b.get("percentage")
            self.t_batt.set(f"{b['voltage']:.1f} V" + (f" · {pct:.0f}%" if pct is not None else ""),
                            theme.BAD if (pct is not None and pct < 25) else theme.TEXT)
        else:
            self.t_batt.set("-")
        if not st.get("mavros_alive", True):
            self.t_link.set("MAVROS DOWN", theme.BAD)
        elif st.get("connected"):
            self.t_link.set("connected", theme.OK)
        else:
            self.t_link.set("waiting FCU", theme.WARN)
        age = st.get("pose_age")
        if age is None:
            self.t_pose.set("none", theme.BAD if armed else theme.WARN)
        else:
            self.t_pose.set(f"{age * 1000:.0f} ms" if age < 1 else f"{age:.1f} s STALE",
                            theme.OK if age < 0.5 else theme.BAD)
        if st.get("locked"):
            self.t_fence.set("LOCKED", theme.BAD)
        elif st.get("last_breach"):
            self.t_fence.set("breach seen", theme.WARN)
        else:
            self.t_fence.set("OK", theme.OK)
        self.b_release.setEnabled(bool(st.get("locked")))

        warn = []
        if st.get("locked"):
            warn.append(f"LOCKED: {st.get('lock_reason')}. Student commands are blocked until you press "
                        f"RELEASE LOCK.")
        if pilot:
            warn.append(f"RC PILOT HAS CONTROL ({pilot}): student commands are ignored and the gate does not "
                        f"enforce its fence. Give it back with ··· -> GUIDED (or a GUIDED switch position).")
        if self.h.square_running:
            last = self.h.square_lines[-1] if self.h.square_lines else "starting"
            warn.append(f"SQUARE DEMO: {last}")
        pw = st.get("param_warnings") or []
        if pw and st.get("params_checked"):
            warn.append(f"{len(pw)} drone parameter warning(s), e.g. {pw[0].split(':')[0]} - see ··· -> "
                        f"Drone parameters.")
        low = st.get("connected") and st.get("pos") is not None and st.get("pose_rate", 20) < 8
        now = time.monotonic()
        self._low_rate_since = (self._low_rate_since or now) if low else None
        if low and now - self._low_rate_since > 6.0:   # ignore the first seconds while the stream ramps up
            warn.append(f"Position arrives at only {st.get('pose_rate')} Hz - fence checks are slow "
                        f"(the gate requests 20 Hz; check the Wi-Fi link / SR0_* parameters).")
        if st.get("executors_alive") is False:
            warn.append("INTERNAL GATE FAILURE - the gate switched to LAND. Disconnect and connect again.")
        if not st.get("verified", True):
            warn.append(f"Fence profile '{st.get('profile_label')}' is marked UNVERIFIED in the config.")
        if not st.get("mavros_alive", True):
            warn.append("MAVROS is not running. Open ··· -> log, then disconnect and connect again.")
        self.banner_text.setText("\n".join(warn))
        self.banner.setVisible(bool(warn))

        self.map.update_state(st.get("fence") or {}, st.get("pos"), st.get("yaw"), bool(st.get("locked")))
        self.i_title.setText(f"Drone {st.get('sysid')} · {st.get('profile_label')}")
        ns = st.get("student_ns", "")
        self.i_ns.setText(f"Students: ROS_DOMAIN_ID={st.get('student_domain')} via {st.get('student_iface', '?')}\n"
                          f"  {ns}/setpoint_position/local\n"
                          f"  {ns}/cmd/arming · {ns}/error\n"
                          f"  {st.get('relayed_topics', 0)} MAVROS topics relayed\n"
                          f"MAVROS: {st.get('fcu_url', '')}\n"
                          f"  private domain {st.get('private_domain')} (loopback only)")
        act = st.get("student_activity") or {}
        if act:
            rows = sorted(act.items(), key=lambda kv: kv[1])[:6]
            self.i_activity.setText("\n".join(f"{v:6.1f}s  {k}" for k, v in rows))
        else:
            self.i_activity.setText("  (no student commands yet)")
        s = st.get("stats") or {}
        lim = st.get("limits") or {}
        self.i_stats.setText(f"accepted {s.get('accepted', 0)} · limited {s.get('modified', 0)} · "
                             f"rejected {s.get('rejected', 0)}\nlimits: {lim.get('max_speed_xy')} m/s horizontal, "
                             f"{lim.get('max_speed_z')} m/s vertical")
        errs = st.get("errors") or []
        key = (len(errs), errs[-1]["t"] if errs else 0, errs[-1]["count"] if errs else 0)
        if key != self._last_err_key:
            self._last_err_key = key
            self.errors.clear()
            for e in reversed(errs):
                t = time.strftime("%H:%M:%S", time.localtime(e["t"]))
                cnt = f" ×{e['count']}" if e.get("count", 1) > 1 else ""
                it = QListWidgetItem(f"{t}  {e['text']}{cnt}")
                if "BREACH" in e["text"] or "LOCK" in e["text"]:
                    it.setForeground(Qt.red)
                self.errors.addItem(it)


# ============================================================================ discovered drone card
class FoundCard(QFrame):
    def __init__(self, app: "FlightWindow", sysid: int):
        super().__init__()
        self.setObjectName("card")
        self.app, self.sysid = app, sysid
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(4)
        top = QHBoxLayout()
        self.l_id = label(f"#{sysid}", "h1")
        self.l_kind = label("", "muted")
        top.addWidget(self.l_id)
        top.addWidget(self.l_kind, 1)
        lay.addLayout(top)
        self.l_state = label("", wrap=True)
        self.l_path = label("", "mono", wrap=True)
        lay.addWidget(self.l_state)
        lay.addWidget(self.l_path)
        self.b = button("CONNECT", "primary", 56)
        self.b.clicked.connect(lambda: self.app.connect_drone(self.sysid))
        lay.addWidget(self.b)

    def refresh(self, row: dict, connected: bool, gate: Optional["GateHandle"] = None):
        self.l_kind.setText(f"{row['autopilot']} {row['type']}")
        if connected:
            st = gate.status if gate else {}
            self.l_state.setText(f"{st.get('mode') or '-'} · connected via the gate")
            self.l_state.setStyleSheet(f"color: {theme.OK};")
            self.l_path.setText(gate.path.describe() if gate and gate.path else (st.get("fcu_url") or ""))
            self.b.setText("CONNECTED")
            self.b.setEnabled(False)
            return
        age = row["age"]
        st = f"{row['mode']} · {'ARMED' if row['armed'] else 'disarmed'}"
        if row["stale"]:
            st += f"  ·  lost {age:.0f}s ago"
        self.l_state.setText(st)
        self.l_state.setStyleSheet(f"color: {theme.BAD if row['stale'] else theme.TEXT};")
        paths = row["paths"]
        p = row["path"]
        txt = (p.describe() if p else "-") + (f"\n+{len(paths) - 1} other path(s)" if len(paths) > 1 else "")
        self.l_path.setText(txt)
        if connected:
            self.b.setText("CONNECTED")
            self.b.setEnabled(False)
        else:
            self.b.setText("CONNECT")
            self.b.setEnabled(p is not None)


# ============================================================================ main window
class FlightWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DroneLab Flight")
        self.cfg = Config.load()
        self.gates: Dict[int, GateHandle] = {}
        self.panels: Dict[int, DronePanel] = {}
        self.cards: Dict[int, FoundCard] = {}
        self.log_lines = []
        self.disc = Discovery(self.cfg.listen_ports, self.cfg.probe_targets, self.cfg.probe_port,
                              self.cfg.probe_local_subnets, self.cfg.probe_interval_s, self.cfg.stale_after_s,
                              log=self.log)
        self._build()
        try:
            self.disc.start()
        except OSError as e:
            self.log(f"[discovery] failed to start: {e}")
        self._attach_existing()
        self.t_fast = QTimer(self)
        self.t_fast.timeout.connect(self._poll_gates)
        self.t_fast.start(200)
        self.t_slow = QTimer(self)
        self.t_slow.timeout.connect(self._poll_discovery)
        self.t_slow.start(1000)
        self._poll_discovery()
        if not shutil.which("ros2"):
            self.log("WARNING: ros2 not found - start the app through the DroneLab launcher (container)")

    # ---------------------------------------------------------------- layout
    def _build(self):
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(14, 10, 14, 10)
        v.setSpacing(10)

        head = QHBoxLayout()
        head.addWidget(label("DRONELAB FLIGHT", "h1"))
        head.addSpacing(18)
        head.addWidget(label("Fence for new connections:", "muted"))
        self.profile = QComboBox()
        self.profile.setMinimumWidth(260)
        self.profile.setMaximumWidth(340)
        self._fill_profiles()
        head.addWidget(self.profile)
        self.l_net = label("", "muted")
        head.addWidget(self.l_net, 1)
        b_menu = button("☰", min_h=52)
        b_menu.setMaximumWidth(70)
        b_menu.clicked.connect(self._menu)
        head.addWidget(b_menu)
        self.b_landall = button("LAND ALL", "danger", 60)
        self.b_landall.setMinimumWidth(170)
        self.b_landall.clicked.connect(self.land_all)
        head.addWidget(self.b_landall)
        v.addLayout(head)

        body = QHBoxLayout()
        left = QVBoxLayout()
        left.addWidget(label("FOUND DRONES", "h2"))
        self.l_scan = label("", "muted", wrap=True)
        left.addWidget(self.l_scan)
        self.found_box = QVBoxLayout()
        self.found_box.setSpacing(8)
        self.found_box.addStretch(1)
        fw = QWidget()
        fw.setObjectName("root")
        fw.setLayout(self.found_box)
        sc = QScrollArea()
        sc.setWidgetResizable(True)
        sc.setWidget(fw)
        sc.viewport().setStyleSheet(f"background: {theme.BG};")
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left.addWidget(sc, 1)
        b_manual = button("Manual connection…", min_h=52)
        b_manual.clicked.connect(self._manual)
        left.addWidget(b_manual)
        lw = QWidget()
        lw.setLayout(left)
        lw.setFixedWidth(330)
        body.addWidget(lw)

        right = QVBoxLayout()
        self.tabs_row = QHBoxLayout()
        self.tabs_row.addStretch(1)
        self.tab_group = QButtonGroup(self)
        self.tab_group.setExclusive(True)
        right.addLayout(self.tabs_row)
        self.stack = QStackedWidget()
        self.empty = label("No drone connected.\n\nPower the drone, wait for it to appear on the left and press "
                           "CONNECT.\nStudents then use /drone<ID>/… on ROS_DOMAIN_ID "
                           f"{self.cfg.student_domain}.", wrap=True)
        self.empty.setAlignment(Qt.AlignCenter)
        self.empty.setStyleSheet(f"color: {theme.MUTED}; font-size: 18px;")
        self.stack.addWidget(self.empty)
        right.addWidget(self.stack, 1)
        body.addLayout(right, 1)
        v.addLayout(body, 1)

        self.l_log = label("", "muted")
        v.addWidget(self.l_log)

    def _fill_profiles(self):
        self.profile.clear()
        self.profile.addItem("drone default", None)
        for name in self.cfg.profile_names():
            p = self.cfg.profiles[name]
            lbl = p.get("label", name) + ("" if p.get("verified", False) else "  (UNVERIFIED)")
            self.profile.addItem(lbl, name)

    def _menu(self):
        d = QDialog(self)
        d.setWindowTitle("Menu")
        d.setMinimumWidth(560)
        lay = QVBoxLayout(d)
        lay.setContentsMargins(20, 20, 20, 20)
        items = (("Open config file", self._open_config), ("Reload config", self._reload),
                 ("Show app log", lambda: self._text_dialog("App log", "\n".join(self.log_lines[-400:]))),
                 ("Open logs folder", lambda: self._xdg(os.path.join(VAR_DIR, "logs"))),
                 ("Quit", self.close), ("Close menu", lambda: None))
        for text, fn in items:
            b = button(text, min_h=60)
            b.clicked.connect(lambda _=False, f=fn: (d.accept(), f()))
            lay.addWidget(b)
        d.exec_()

    def _xdg(self, path):
        for tool in ("xdg-open", "kate", "gedit", "mousepad"):
            if shutil.which(tool):
                subprocess.Popen([tool, path], start_new_session=True)
                return
        self._text_dialog(path, open(path).read() if os.path.isfile(path) else path)

    def _open_config(self):
        self._xdg(self.cfg.path)

    def _reload(self):
        try:
            self.cfg = Config.load()
        except Exception as e:
            self._text_dialog("Config error", f"The config was NOT reloaded:\n\n{e}")
            return
        self._fill_profiles()
        self.log("config reloaded (applies to new connections; reconnect a drone to apply it there)")

    def _text_dialog(self, title, text):
        d = QDialog(self)
        d.setWindowTitle(title)
        d.resize(1100, 680)
        lay = QVBoxLayout(d)
        t = QPlainTextEdit(text)
        t.setReadOnly(True)
        lay.addWidget(t)
        b = button("Close", min_h=56)
        b.clicked.connect(d.accept)
        lay.addWidget(b)
        t.verticalScrollBar().setValue(t.verticalScrollBar().maximum())
        d.exec_()

    def log(self, msg: str):
        line = time.strftime("%H:%M:%S ") + msg
        self.log_lines.append(line)
        print(line, flush=True)
        if hasattr(self, "l_log"):
            self.l_log.setText(line)

    # ---------------------------------------------------------------- discovery
    def _poll_discovery(self):
        ips = ", ".join(str(i.ip) for i in self.disc.local_ifaces) or "no network"
        self.l_net.setText(f"Deck: {ips}")
        busy = self.disc.busy_ports
        self.l_scan.setText(f"listening on {len(self.disc.listen_ports) - len(busy)} UDP ports, probing "
                            f"{len(self.cfg.probe_targets)} hosts + local /24 on :{self.cfg.probe_port}")
        rows = self.disc.snapshot()
        seen = set()
        for row in rows:
            sid = row["sysid"]
            seen.add(sid)
            if row["stale"] and row["age"] > 120 and sid not in self.gates:
                self.disc.forget(sid)
                continue
            c = self.cards.get(sid)
            if c is None:
                c = FoundCard(self, sid)
                self.cards[sid] = c
                self.found_box.insertWidget(self.found_box.count() - 1, c)
            c.refresh(row, sid in self.gates, self.gates.get(sid))
        for sid in list(self.cards):
            if sid not in seen:
                self.cards.pop(sid).deleteLater()

    # ---------------------------------------------------------------- connect / disconnect
    def _profile_choice(self, sysid: int) -> Optional[str]:
        name = self.profile.currentData()
        return name

    def connect_drone(self, sysid: int, fcu_url: Optional[str] = None):
        if sysid in self.gates:
            return
        prof = self._profile_choice(sysid)
        try:
            ds = self.cfg.drone(sysid, prof)
        except ValueError as e:
            self._text_dialog("Cannot connect", str(e))
            return
        if not ds.profile_verified and not Confirm.ask(
                self, "Unverified fence", f"Fence profile '{ds.profile_label}' is marked verified: false in the "
                f"config. Check the coordinates/limits before flying.\n\nConnect anyway?", "Connect anyway"):
            return
        path = None
        if fcu_url is None:
            row = next((r for r in self.disc.snapshot() if r["sysid"] == sysid), None)
            if not row or not row["path"]:
                self.log(f"drone {sysid}: no fresh path")
                return
            path = row["path"]
            local = procman.free_udp_port(14600 + sysid) if path.via == "probe" else None
            fcu_url = path.fcu_url(local)
            self.disc.release(path)
        try:
            proc = procman.spawn_gate(sysid, fcu_url, prof)
        except OSError as e:
            self._text_dialog("Cannot start gate", str(e))
            if path:
                self.disc.reclaim(path)
            return
        self.log(f"drone {sysid}: starting gate + MAVROS ({fcu_url}, fence {ds.profile_label})")
        self._add_gate(GateHandle(sysid, ds.socket_path, proc, path))

    def _add_gate(self, h: GateHandle):
        self.gates[h.sysid] = h
        panel = DronePanel(self, h)
        self.panels[h.sysid] = panel
        self.stack.addWidget(panel)
        tb = button(f"#{h.sysid}", "tab", 48)
        tb.setCheckable(True)
        tb.clicked.connect(lambda: self.stack.setCurrentWidget(panel))
        self.tab_group.addButton(tb, h.sysid)
        self.tabs_row.insertWidget(self.tabs_row.count() - 1, tb)
        tb.setChecked(True)
        self.stack.setCurrentWidget(panel)

    def _remove_gate(self, sysid: int):
        h = self.gates.pop(sysid, None)
        p = self.panels.pop(sysid, None)
        tb = self.tab_group.button(sysid)
        if tb:
            self.tab_group.removeButton(tb)
            tb.deleteLater()
        if p:
            self.stack.removeWidget(p)
            p.deleteLater()
        if h:
            h.client.close()
            if h.path:
                self.disc.reclaim(h.path)
        if not self.panels:
            self.stack.setCurrentWidget(self.empty)

    def _attach_existing(self):
        for sysid, sock in procman.existing_gate_sockets().items():
            c = AdminClient(sock)
            if c.connect():
                c.close()
                self.log(f"drone {sysid}: re-attached to running gate")
                self._add_gate(GateHandle(sysid, sock))
            else:
                try:
                    os.unlink(sock)
                except OSError:
                    pass

    def disconnect(self, h: GateHandle):
        st = h.status
        if st.get("armed") and not Confirm.ask(
                self, "Drone is ARMED", "Disconnecting stops MAVROS and the fence protection for this drone. "
                "Only do this if you have it under RC control.", "Disconnect", danger=True):
            return
        h.client.send("shutdown", force=True)
        if h.proc is not None:
            QTimer.singleShot(100, lambda: procman.kill_group(h.proc, timeout=10.0))
        self.log(f"drone {h.sysid}: disconnecting")
        self._remove_gate(h.sysid)

    def admin(self, h: GateHandle, cmd: str, **kw):
        if h.client.send(cmd, **kw) is None:
            self.log(f"drone {h.sysid}: gate not reachable for '{cmd}'")
        else:
            self.log(f"drone {h.sysid}: {cmd} {kw if kw else ''}")

    # ---------------------------------------------------------------- square demo
    def fly_square(self, h: GateHandle):
        st = h.status
        if not st.get("connected"):
            self._text_dialog("Square demo", "The drone is not connected.")
            return
        if st.get("locked") or st.get("pilot_control"):
            self._text_dialog("Square demo", "The drone is locked or the RC pilot has control - release / hand it "
                                             "back first.")
            return
        zmax = float((st.get("fence") or {}).get("z", [0, 3.3])[1])
        alt = min(3.0, zmax - 0.3)
        lim = (st.get("limits") or {}).get("max_speed_xy", 1.0)
        note = "" if alt >= 3.0 else f"\n\n3 m does not fit under the fence ceiling ({zmax:g} m): it flies at {alt:.1f} m."
        what = "take off to" if not st.get("armed") else "go from its current position to"
        if not Confirm.ask(self, f"Square demo - drone {h.sysid}",
                           f"The drone will {what} {alt:.1f} m, fly a 3 x 3 m square at {min(1.0, lim):g} m/s "
                           f"(placed where it fits in the fence), come back and LAND.{note}\n\nMake sure the area "
                           f"is clear. Take over at any time with the RC mode switch (LOITER): the demo stops "
                           f"immediately.", "Fly square", danger=True):
            return
        cmd = [sys.executable, "-m", "dronelab.demos.square", "--drone", str(h.sysid), "--yes"]
        env = dict(os.environ)
        env.pop("CYCLONEDDS_URI", None)
        repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        try:
            h.square = subprocess.Popen(cmd, cwd=repo, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, start_new_session=True)
        except OSError as e:
            self._text_dialog("Square demo", str(e))
            return
        h.square_lines.clear()
        logf = procman.log_path(f"square-{h.sysid}.log")

        def pump(p=h.square, lines=h.square_lines, sysid=h.sysid):
            with open(logf, "w") as f:
                for line in p.stdout:
                    line = line.rstrip()
                    lines.append(line.split(" ", 1)[-1] if line[:2].isdigit() else line)
                    f.write(line + "\n")
                    f.flush()
            lines.append(f"finished (exit {p.wait()})")
        threading.Thread(target=pump, daemon=True).start()
        self.log(f"drone {h.sysid}: square demo started")

    def stop_square(self, h: GateHandle):
        if h.square_running:
            h.square.send_signal(signal.SIGINT)        # the demo lands if it still flies GUIDED
            self.log(f"drone {h.sysid}: square demo stopped (landing)")

    # ---------------------------------------------------------------- drone parameters
    def params_dialog(self, h: GateHandle):
        st = h.status
        d = QDialog(self)
        d.setWindowTitle(f"Drone {h.sysid} parameters")
        d.resize(1100, 700)
        lay = QVBoxLayout(d)
        lay.setContentsMargins(20, 20, 20, 20)
        lay.addWidget(label(f"Drone {h.sysid} - RC takeover and fail-safe parameters "
                            f"({st.get('profile_label', '')})", "h2"))
        review = st.get("param_review") or []
        if not st.get("params_checked"):
            text = "Parameters not read yet (MAVROS loads them a few seconds after connecting)."
        elif not review:
            text = "Everything looks fine."
        else:
            text = "\n\n".join(("WARNING  " if f["level"] == "warn" else "note     ") + f["text"] for f in review)
        fixes = {}
        for f in review:
            fixes.update(f.get("fix") or {})
        p = st.get("params") or {}
        if fixes:
            text += "\n\n---------- 'Fix on the drone' writes (stored permanently) ----------\n" + "\n".join(
                f"  {k:<15} {p.get(k, '?'):g} -> {v:g}" if isinstance(p.get(k), (int, float)) else f"  {k} -> {v:g}"
                for k, v in fixes.items())
        text += ("\n\nTakeover in flight: move the flight-mode switch (LOITER). The gate then ignores the students "
                 "and does not enforce its fence. Give the drone back with ··· -> GUIDED.\n"
                 "The same values for Mission Planner: drones/params/outdoor-rc-takeover.param and "
                 "indoor-rc-takeover.param.")
        t = QPlainTextEdit(text)
        t.setReadOnly(True)
        lay.addWidget(t, 1)
        row = QHBoxLayout()
        b_check = button("Read again", min_h=56)
        b_check.clicked.connect(lambda: (self.admin(h, "check_params"), d.accept()))
        b_fix = button("Fix on the drone", "primary", 56)
        b_fix.setEnabled(bool(fixes) and not st.get("armed"))
        if st.get("armed"):
            b_fix.setText("Fix on the drone (disarm first)")

        def fix():
            if Confirm.ask(self, "Write parameters?", "Write these values to the flight controller now?\n\n"
                           + "\n".join(f"{k} = {v:g}" for k, v in fixes.items()), "Write", danger=False):
                d.accept()
                self.admin(h, "fix_params")
        b_fix.clicked.connect(fix)
        b_close = button("Close", min_h=56)
        b_close.clicked.connect(d.reject)
        for b in (b_check, b_fix, b_close):
            row.addWidget(b)
        lay.addLayout(row)
        d.exec_()

    def land_all(self):
        for h in self.gates.values():
            self.admin(h, "land")
        if not self.gates:
            self.log("LAND ALL: no connected drones")

    def show_log(self, h: GateHandle):
        txt = procman.tail(procman.log_path(f"gate-{h.sysid}.log"), 120)
        txt += "\n\n----- MAVROS -----\n" + procman.tail(procman.log_path(f"mavros-{h.sysid}.log"), 200)
        self._text_dialog(f"Drone {h.sysid} log", txt)

    def _manual(self):
        d = QDialog(self)
        d.setWindowTitle("Manual connection")
        d.setMinimumWidth(640)
        lay = QFormLayout(d)
        sid = QSpinBox()
        sid.setRange(1, 255)
        sid.setValue(10)
        url = QLineEdit("udp://0.0.0.0:14550@")
        lay.addRow(label("Use this when discovery cannot see the drone (serial, TCP, unusual routing).", wrap=True))
        lay.addRow("System ID", sid)
        lay.addRow("MAVROS fcu_url", url)
        lay.addRow(label("Examples: udp://0.0.0.0:14513@   udp://0.0.0.0:14600@10.42.0.1:14550   "
                         "tcp://192.168.18.111:5760   /dev/ttyACM0:115200", "mono", wrap=True))
        row = QHBoxLayout()
        b_ok, b_no = button("Connect", "primary", 56), button("Cancel", min_h=56)
        b_ok.clicked.connect(d.accept)
        b_no.clicked.connect(d.reject)
        row.addWidget(b_no)
        row.addWidget(b_ok)
        lay.addRow(row)
        if d.exec_() == QDialog.Accepted:
            self.connect_drone(sid.value(), url.text().strip())

    # ---------------------------------------------------------------- polling
    def _poll_gates(self):
        for sysid, h in list(self.gates.items()):
            st = h.client.poll()
            if st:
                h.status = st
            for r in h.client.pop_replies():
                self.log(f"drone {sysid}: {r.get('cmd')} -> {'OK' if r.get('ok') else 'FAILED'} {r.get('msg', '')}")
            if h.proc is not None and h.proc.poll() is not None:
                self.log(f"drone {sysid}: gate exited (code {h.proc.returncode}) - see ··· -> log")
                tail = procman.tail(procman.log_path(f"gate-{sysid}.log"), 15)
                self._remove_gate(sysid)
                self._text_dialog(f"Drone {sysid}: gate stopped", tail)
                continue
            fresh = h.client.connected and time.monotonic() - h.client.last_status_time < 2.0
            if not fresh and h.proc is None and not os.path.exists(h.client.path):
                self.log(f"drone {sysid}: gate is gone")
                self._remove_gate(sysid)
                continue
            if not fresh and time.monotonic() - h.started > 20 and h.status:
                h.status["connected"] = False
            self.panels[sysid].refresh(h.status, h.alive)
            tb = self.tab_group.button(sysid)
            if tb:
                st = h.status
                mark = "LOCKED" if st.get("locked") else ("ARMED" if st.get("armed") else "")
                tb.setText(f"#{sysid} {st.get('mode', '')} {mark}".strip())

    # ---------------------------------------------------------------- shutdown
    def closeEvent(self, e):
        if self.gates:
            d = QDialog(self)
            d.setWindowTitle("Quit")
            lay = QVBoxLayout(d)
            lay.setContentsMargins(20, 20, 20, 20)
            lay.addWidget(label(f"{len(self.gates)} drone(s) connected.", "h2"))
            lay.addWidget(label("Keeping the gates running keeps the fence active and lets students continue. "
                                "Reopen the app to control them again.", wrap=True))
            choice = {"v": None}
            for text, val, obj in (("Keep gates running and quit", "keep", "primary"),
                                   ("Stop all gates and quit", "stop", "danger"), ("Cancel", None, "")):
                b = button(text, obj, 60)
                b.clicked.connect(lambda _=False, v=val: (choice.update(v=v), d.accept()))
                lay.addWidget(b)
            d.exec_()
            if choice["v"] is None:
                e.ignore()
                return
            if choice["v"] == "stop":
                flying = [h.sysid for h in self.gates.values() if h.status.get("armed")]
                if flying and not Confirm.ask(self, "Drones are ARMED",
                                              f"Drone(s) {flying} are armed. Stopping their gates stops MAVROS "
                                              f"and the fence. Only do this with the RC in hand.", "Stop anyway",
                                              danger=True):
                    e.ignore()
                    return
                for h in list(self.gates.values()):
                    h.client.send("shutdown", force=True)
                    if h.proc is not None:
                        procman.kill_group(h.proc, timeout=10.0)
        for h in self.gates.values():
            self.stop_square(h)          # never leave an autopilot script running without its window
        self.disc.stop()
        e.accept()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--windowed", action="store_true")
    a, rest = ap.parse_known_args(argv)
    app = QApplication(sys.argv[:1] + rest)
    app.setStyleSheet(theme.QSS)
    w = FlightWindow()
    w.resize(1280, 800)
    if a.windowed:
        w.show()
    else:
        w.showMaximized()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
