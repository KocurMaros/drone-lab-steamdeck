"""DroneLab Demo: fly the simulated drone with a gamepad, FPV camera + HUD, minimap and a gate race.

    python3 -m dronelab.gui.demo_app [--windowed] [--no-sim] [--gazebo on|off|auto]

--no-sim attaches to an already running simulation (e.g. started by hand).
"""
from __future__ import annotations

import argparse
import collections
import math
import sys
import threading
import time
from typing import Optional

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import (QApplication, QDialog, QGridLayout, QHBoxLayout, QMainWindow, QSizePolicy, QVBoxLayout,
                             QWidget)

from ..config import Config
from ..gate.admin import AdminClient
from ..sim.race import Course, Leaderboard, Race, fmt_time
from ..sim.stack import SimStack
from . import theme
from .gamepad import Gamepad, KeyboardSticks
from .widgets import Confirm, button, card, label

NS = "/drone1"
TYPE_MASK_BODY_VEL = 1 | 2 | 4 | 64 | 128 | 256 | 1024   # use velocity + yaw rate
CAM_PITCH = 0.26        # rad, camera tilted down (tools/gen_arena.py)
CAM_HFOV = 1.75


# ============================================================================ ROS link
class RosLink:
    """All ROS traffic of the demo app (student side of the gate + Gazebo services)."""

    def __init__(self, domain: int, camera_topic: str):
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from mavros_msgs.msg import ExtendedState, PositionTarget, State, StatusText
        from sensor_msgs.msg import BatteryState, Image
        from std_msgs.msg import String

        self.PositionTarget = PositionTarget
        self.ctx = Context()
        self.ctx.init(domain_id=domain)
        self.node = Node("dronelab_demo", context=self.ctx)
        be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        be1 = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock = threading.Lock()
        self.image = None
        self.image_t = 0.0
        self.image_count = 0
        self.pose = None
        self.pose_t = 0.0
        self.vel = (0.0, 0.0, 0.0)
        self.state = None
        self.state_t = 0.0
        self.landed = 0
        self.battery = None
        self.texts = collections.deque(maxlen=20)
        self.gate_errors = collections.deque(maxlen=20)
        n = self.node
        n.create_subscription(Image, camera_topic, self._on_image, be1)
        n.create_subscription(PoseStamped, f"{NS}/local_position/pose", self._on_pose, be)
        n.create_subscription(TwistStamped, f"{NS}/local_position/velocity_local", self._on_vel, be)
        n.create_subscription(State, f"{NS}/state", self._on_state, be)
        n.create_subscription(ExtendedState, f"{NS}/extended_state", self._on_ext, be)
        n.create_subscription(BatteryState, f"{NS}/battery", self._on_batt, be)
        n.create_subscription(StatusText, f"{NS}/statustext/recv", self._on_text, be)
        n.create_subscription(String, f"{NS}/error", self._on_err, be)
        self.pub = n.create_publisher(PositionTarget, f"{NS}/setpoint_raw/local", 10)
        try:
            from gazebo_msgs.srv import SetEntityState
            self.SetEntityState = SetEntityState
            self.cli_set = n.create_client(SetEntityState, "/gazebo/set_entity_state")
        except ImportError:
            self.SetEntityState = None
            self.cli_set = None
        self.ex = MultiThreadedExecutor(num_threads=2, context=self.ctx)
        self.ex.add_node(n)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            self.ex.spin()
        except Exception:
            pass

    def close(self):
        try:
            self.ex.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            self.ctx.try_shutdown()
        except Exception:
            pass

    # ---- callbacks (executor thread)
    def _on_image(self, m):
        if m.encoding not in ("rgb8", "bgr8"):
            return
        with self.lock:
            self.image = (bytes(m.data), m.width, m.height, m.step, m.encoding)
            self.image_t = time.monotonic()
            self.image_count += 1

    def _on_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.pose = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            self.pose_t = time.monotonic()

    def _on_vel(self, m):
        v = m.twist.linear
        with self.lock:
            self.vel = (v.x, v.y, v.z)

    def _on_state(self, m):
        with self.lock:
            self.state = (m.connected, m.armed, m.mode)
            self.state_t = time.monotonic()

    def _on_ext(self, m):
        with self.lock:
            self.landed = m.landed_state

    def _on_batt(self, m):
        with self.lock:
            self.battery = (m.voltage, m.percentage)

    def _on_text(self, m):
        with self.lock:
            self.texts.append((time.monotonic(), m.text))

    def _on_err(self, m):
        with self.lock:
            self.gate_errors.append((time.monotonic(), m.data))

    # ---- commands
    def send_body_velocity(self, vx, vy, vz, yaw_rate):
        msg = self.PositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.coordinate_frame = 8   # FRAME_BODY_NED: MAVROS converts our FLU values
        msg.type_mask = TYPE_MASK_BODY_VEL
        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(vx), float(vy), float(vz)
        msg.yaw_rate = float(yaw_rate)
        self.pub.publish(msg)

    def teleport(self, x_enu, y_enu, z=0.05, timeout=3.0) -> bool:
        if self.cli_set is None or not self.cli_set.wait_for_service(timeout_sec=1.0):
            return False
        req = self.SetEntityState.Request()
        req.state.name = "dronelab_iris"
        req.state.pose.position.x = float(y_enu)    # gazebo x = north
        req.state.pose.position.y = float(-x_enu)   # gazebo y = west
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = "world"
        fut = self.cli_set.call_async(req)
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        return ev.wait(timeout) and bool(fut.result() and fut.result().success)


def quat_to_rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rotate_world_to_body(q, v):
    """v (world ENU) -> body FLU using the inverse of quaternion q (body->world)."""
    x, y, z, w = q
    # conjugate rotation
    qx, qy, qz, qw = -x, -y, -z, w
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty), vy + qw * ty + (qz * tx - qx * tz), vz + qw * tz + (qx * ty - qy * tx))


# ============================================================================ FPV view with HUD
class FpvView(QWidget):
    def __init__(self, win: "DemoWindow"):
        super().__init__()
        self.win = win
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.qimg: Optional[QImage] = None
        self.setMinimumSize(640, 480)

    def set_image(self, data, w, h, step, enc):
        fmt = QImage.Format_RGB888 if enc == "rgb8" else QImage.Format_BGR888
        self.qimg = QImage(data, w, h, step, fmt).copy()

    def _project(self, rect: QRectF, pos, q, target):
        """Project a world point into the image rect; returns QPointF or None if behind."""
        v = (target[0] - pos[0], target[1] - pos[1], target[2] - pos[2])
        b = rotate_world_to_body(q, v)
        cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
        fwd = b[0] * cp - b[2] * sp
        up = b[0] * sp + b[2] * cp
        left = b[1]
        if fwd < 0.5:
            return None
        f = (rect.width() / 2) / math.tan(CAM_HFOV / 2)
        return QPointF(rect.center().x() - f * left / fwd, rect.center().y() - f * up / fwd), fwd

    def paintEvent(self, _e):
        w = self.win
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#05080b"))
        R = QRectF(self.rect())
        if self.qimg is not None:
            iw, ih = self.qimg.width(), self.qimg.height()
            s = min(R.width() / iw, R.height() / ih)
            R = QRectF((self.width() - iw * s) / 2, (self.height() - ih * s) / 2, iw * s, ih * s)
            p.drawImage(R, self.qimg)
        else:
            p.setPen(QColor(theme.MUTED))
            f = QFont()
            f.setPointSize(18)
            p.setFont(f)
            p.drawText(self.rect(), Qt.AlignCenter, "waiting for the drone camera…")
        tel = w.tel
        big = QFont()
        big.setPointSize(22)
        big.setBold(True)
        small = QFont()
        small.setPointSize(13)
        small.setBold(True)
        white = QColor(255, 255, 255)
        shadow = QColor(0, 0, 0, 170)

        def text(x, y, s, font=small, color=white, align=Qt.AlignLeft, wdt=300):
            p.setFont(font)
            r = QRectF(x, y, wdt, 40) if align == Qt.AlignLeft else QRectF(x - wdt, y, wdt, 40)
            if align == Qt.AlignHCenter:
                r = QRectF(x - wdt / 2, y, wdt, 40)
            p.setPen(shadow)
            p.drawText(r.translated(2, 2), align | Qt.AlignVCenter, s)
            p.setPen(color)
            p.drawText(r, align | Qt.AlignVCenter, s)

        if tel["pose"] is not None:
            pos, q = tel["pose"]
            roll, pitch, yaw = quat_to_rpy(q)
            # next-gate bracket
            race = w.race
            if w.race_mode and not race.all_gates and w.state in ("READY", "STARTING", "FLYING", "LANDING"):
                g = w.course.gates[race.next_gate]
                pr = self._project(R, pos, q, (g.x, g.y, g.z))
                col = QColor(g.qcolor_hex)
                if pr:
                    c, dist = pr
                    f = (R.width() / 2) / math.tan(CAM_HFOV / 2)
                    half = max(14.0, min(R.width() / 2, f * g.width / 2 / dist))
                    p.setPen(QPen(col, 4))
                    p.setBrush(Qt.NoBrush)
                    L = half * 0.35
                    for sx in (-1, 1):
                        for sy in (-1, 1):
                            cx, cy = c.x() + sx * half, c.y() + sy * half
                            p.drawLine(QPointF(cx, cy), QPointF(cx - sx * L, cy))
                            p.drawLine(QPointF(cx, cy), QPointF(cx, cy - sy * L))
                    lx = min(max(c.x(), R.left() + 130), R.right() - 130)
                    ly = min(c.y() + half + 6, R.bottom() - 40)
                    text(lx, ly, f"GATE {g.idx + 1}  {dist:.0f} m", small, col, Qt.AlignHCenter, 260)
                else:
                    # arrow on the screen edge toward the gate
                    bearing = math.atan2(g.y - pos[1], g.x - pos[0]) - yaw
                    bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
                    side = -1 if bearing > 0 else 1   # left if positive (CCW)
                    cx = R.center().x() + side * (R.width() / 2 - 60)
                    cy = R.center().y()
                    p.setBrush(col)
                    p.setPen(QPen(QColor(0, 0, 0), 2))
                    tri = QPolygonF([QPointF(cx + side * 30, cy), QPointF(cx - side * 10, cy - 28),
                                     QPointF(cx - side * 10, cy + 28)])
                    p.drawPolygon(tri)
                    text(cx, cy + 34, f"GATE {g.idx + 1}", small, col, Qt.AlignHCenter, 160)
            # crosshair
            cx, cy = R.center().x(), R.center().y() - R.height() * 0.0
            p.setPen(QPen(QColor(255, 255, 255, 200), 2))
            p.drawLine(QPointF(cx - 18, cy), QPointF(cx - 6, cy))
            p.drawLine(QPointF(cx + 6, cy), QPointF(cx + 18, cy))
            p.drawLine(QPointF(cx, cy - 18), QPointF(cx, cy - 6))
            # heading tape
            hdg = (90 - math.degrees(yaw)) % 360
            tape_w = min(420, R.width() * 0.5)
            x0 = R.center().x() - tape_w / 2
            y0 = R.top() + 12
            p.fillRect(QRectF(x0, y0, tape_w, 34), QColor(0, 0, 0, 110))
            p.setFont(small)
            for d in range(-60, 61, 5):
                a = (hdg + d) % 360
                xx = R.center().x() + d * tape_w / 120
                rounded = round(a / 5) * 5 % 360
                if abs(a - rounded) > 2.5 and abs(a - rounded) < 357.5:
                    continue
                p.setPen(QColor(255, 255, 255, 220))
                p.drawLine(QPointF(xx, y0 + 26), QPointF(xx, y0 + 34))
                names = {0: "N", 90: "E", 180: "S", 270: "W"}
                if rounded % 45 == 0:
                    lab = names.get(rounded, f"{rounded}")
                    p.drawText(QRectF(xx - 30, y0, 60, 24), Qt.AlignCenter, lab)
            p.setPen(QPen(QColor(theme.WARN), 3))
            p.drawLine(QPointF(R.center().x(), y0), QPointF(R.center().x(), y0 + 34))
            # altitude & speed
            spd = math.hypot(tel["vel"][0], tel["vel"][1])
            yb = R.bottom() - (96 if tel["fence_warn"] else 56)
            text(R.left() + 18, yb - 26, "ALT", small, QColor(theme.MUTED))
            text(R.left() + 18, yb, f"{pos[2]:4.1f} m", big)
            text(R.right() - 18, yb - 26, "SPEED", small, QColor(theme.MUTED), Qt.AlignRight)
            text(R.right() - 18, yb, f"{spd:4.1f} m/s", big, white, Qt.AlignRight)
        # top-left: timer / gates
        if w.race_mode:
            t = w.race.elapsed(time.monotonic())
            text(R.left() + 16, R.top() + 10, fmt_time(t), big, QColor(theme.WARN) if w.race.active else white)
            text(R.left() + 16, R.top() + 46, f"GATES {w.race.next_gate}/{len(w.course.gates)}", small)
        else:
            text(R.left() + 16, R.top() + 10, "FREE FLIGHT", small)
        # top-right: mode / battery
        st = tel["state"]
        mode = st[2] if st else "-"
        armed = st[1] if st else False
        text(R.right() - 16, R.top() + 10, f"{mode}  {'ARMED' if armed else 'SAFE'}", small,
             QColor(theme.WARN) if armed else white, Qt.AlignRight)
        if tel["battery"]:
            text(R.right() - 16, R.top() + 36, f"{tel['battery'][0]:.1f} V", small, white, Qt.AlignRight)
        if tel["fence_warn"]:
            p.fillRect(QRectF(R.left(), R.bottom() - 40, R.width(), 40), QColor(255, 60, 60, 120))
            text(R.center().x(), R.bottom() - 40, "INVISIBLE FENCE - the safety gate is slowing you down", small,
                 white, Qt.AlignHCenter, R.width())
        # big centre message
        msg, color = w.overlay()
        if msg:
            f = QFont()
            f.setPointSize(30)
            f.setBold(True)
            p.setFont(f)
            lines = msg.split("\n")
            hh = 54 * len(lines) + 20
            box = QRectF(R.center().x() - R.width() * 0.42, R.center().y() - hh / 2 + R.height() * 0.18,
                         R.width() * 0.84, hh)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 150))
            p.drawRoundedRect(box, 16, 16)
            p.setPen(QColor(color))
            p.drawText(box, Qt.AlignCenter, msg)


# ============================================================================ minimap
class MiniMap(QWidget):
    def __init__(self, win: "DemoWindow"):
        super().__init__()
        self.win = win
        self.trail = collections.deque(maxlen=400)
        self.setMinimumSize(360, 360)

    def paintEvent(self, _e):
        w, c = self.win, self.win.course
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#16301c"))
        (x0, x1), (y0, y1) = c.fence["x"], c.fence["y"]
        pad = 4.0
        sx = (self.width() - 16) / (x1 - x0 + 2 * pad)
        sy = (self.height() - 16) / (y1 - y0 + 2 * pad)
        s = min(sx, sy)
        ox = self.width() / 2 - s * (x0 + x1) / 2
        oy = self.height() / 2 + s * (y0 + y1) / 2

        def T(x, y):
            return QPointF(ox + s * x, oy - s * y)

        # fence
        p.setPen(QPen(QColor(theme.BAD), 2, Qt.DashLine))
        p.setBrush(QColor(40, 90, 50))
        p.drawRect(QRectF(T(x0, y1), T(x1, y0)))
        # roads (clipped to the fence area)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(78, 78, 82))
        for rd in c.roads:
            rx0, rx1 = max(x0 - pad, rd["x"] - rd["w"] / 2), min(x1 + pad, rd["x"] + rd["w"] / 2)
            ry0, ry1 = max(y0 - pad, rd["y"] - rd["d"] / 2), min(y1 + pad, rd["y"] + rd["d"] / 2)
            p.drawRect(QRectF(T(rx0, ry1), T(rx1, ry0)))
        # buildings / trees / towers
        for b in c.buildings:
            p.setBrush(QColor(150, 110, 90))
            p.setPen(QPen(QColor(40, 30, 20), 1))
            p.drawRect(QRectF(T(b["x"] - b["w"] / 2, b["y"] + b["d"] / 2), T(b["x"] + b["w"] / 2, b["y"] - b["d"] / 2)))
        p.setPen(Qt.NoPen)
        for t in c.trees:
            p.setBrush(QColor(30, 110, 40))
            p.drawEllipse(T(t[0], t[1]), 2.0 * s, 2.0 * s)
        for t in c.towers:
            p.setBrush(QColor(230, 230, 230))
            p.drawEllipse(T(t["x"], t["y"]), max(3.0, t["r"] * s), max(3.0, t["r"] * s))
        # pad
        px, py, pr = c.pad
        p.setBrush(QColor(60, 60, 60))
        p.setPen(QPen(QColor(theme.WARN), 2))
        p.drawEllipse(T(px, py), pr * s, pr * s)
        p.setPen(QColor(255, 255, 255))
        f = QFont()
        f.setPointSize(9)
        f.setBold(True)
        p.setFont(f)
        p.drawText(QRectF(T(px, py).x() - 10, T(px, py).y() - 10, 20, 20), Qt.AlignCenter, "H")
        # gates
        for g in c.gates:
            col = QColor(g.qcolor_hex)
            done = w.race_mode and g.idx < w.race.next_gate
            nxt = w.race_mode and g.idx == w.race.next_gate
            if done:
                col.setAlpha(90)
            half = g.width / 2 + 0.6
            lx, ly = -math.sin(g.yaw) * half, math.cos(g.yaw) * half
            p.setPen(QPen(col, 7 if nxt else 4, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(T(g.x - lx, g.y - ly), T(g.x + lx, g.y + ly))
            ax, ay = math.cos(g.yaw) * 2.5, math.sin(g.yaw) * 2.5
            p.setPen(QPen(col, 2))
            p.drawLine(T(g.x, g.y), T(g.x + ax, g.y + ay))
            p.setPen(QColor(255, 255, 255) if not done else QColor(255, 255, 255, 90))
            f.setPointSize(11 if nxt else 9)
            p.setFont(f)
            c0 = T(g.x - math.cos(g.yaw) * 3.2, g.y - math.sin(g.yaw) * 3.2)
            p.drawText(QRectF(c0.x() - 12, c0.y() - 12, 24, 24), Qt.AlignCenter, str(g.idx + 1))
        # trail + drone
        tel = w.tel
        if tel["pose"] is not None:
            pos, q = tel["pose"]
            if not self.trail or math.dist(self.trail[-1], pos[:2]) > 0.3:
                self.trail.append(pos[:2])
            if len(self.trail) > 1:
                path = QPainterPath(T(*self.trail[0]))
                for pt in list(self.trail)[1:]:
                    path.lineTo(T(*pt))
                p.setPen(QPen(QColor(120, 200, 255, 160), 2))
                p.setBrush(Qt.NoBrush)
                p.drawPath(path)
            _, _, yaw = quat_to_rpy(q)
            cpt = T(pos[0], pos[1])
            L = 13
            pts = [QPointF(cpt.x() + L * math.cos(yaw), cpt.y() - L * math.sin(yaw)),
                   QPointF(cpt.x() + 0.7 * L * math.cos(yaw + 2.5), cpt.y() - 0.7 * L * math.sin(yaw + 2.5)),
                   QPointF(cpt.x() + 0.7 * L * math.cos(yaw - 2.5), cpt.y() - 0.7 * L * math.sin(yaw - 2.5))]
            p.setBrush(QColor(theme.WARN))
            p.setPen(QPen(QColor(0, 0, 0), 1.5))
            p.drawPolygon(QPolygonF(pts))
        p.setPen(QColor(255, 255, 255, 150))
        f.setPointSize(9)
        p.setFont(f)
        p.drawText(QRectF(8, 4, 40, 18), Qt.AlignLeft, "N ↑")


class GateDots(QWidget):
    def __init__(self, win: "DemoWindow"):
        super().__init__()
        self.win = win
        self.setFixedHeight(34)

    def paintEvent(self, _e):
        w = self.win
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        n = len(w.course.gates)
        step = min(40.0, (self.width() - 10) / max(n, 1))
        f = QFont()
        f.setPointSize(9)
        f.setBold(True)
        p.setFont(f)
        for g in w.course.gates:
            cx = 5 + step * g.idx + step / 2
            col = QColor(g.qcolor_hex)
            done = g.idx < w.race.next_gate
            nxt = g.idx == w.race.next_gate
            p.setPen(QPen(col, 3))
            p.setBrush(col if done else QColor(theme.PANEL))
            r = 13 if nxt else 11
            p.drawEllipse(QPointF(cx, 17), r, r)
            p.setPen(QColor("#000000") if done else col)
            p.drawText(QRectF(cx - 12, 5, 24, 24), Qt.AlignCenter, str(g.idx + 1))


# ============================================================================ main window
class DemoWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("DroneLab Demo")
        self.cfg = Config.load()
        d = self.cfg.data.get("demo", {}) or {}
        self.d = d
        self.course = Course.load()
        self.race = Race(self.course)
        self.board = Leaderboard()
        self.race_mode = True
        self.expert = False
        self.takeoff_alt = float(d.get("takeoff_alt", 2.5))
        self.skill = d.get("skill", {}) or {}
        self.state = "BOOT"
        self.state_t = time.monotonic()
        self.flash = ("", theme.TEXT, 0.0)
        self.pilot = ""
        self.tel = {"pose": None, "vel": (0, 0, 0), "state": None, "battery": None, "fence_warn": False}
        self.pad = Gamepad()
        self.keys = KeyboardSticks()
        self.stack = SimStack(self.cfg, log=print)
        self.admin = AdminClient(self.cfg.drone(1).socket_path)
        self.ros: Optional[RosLink] = None
        self.ros_error = ""
        self._busy = False
        self._crash_t = None
        self._last_armed = False
        self._was_flying = False
        self._build()
        try:
            self.ros = RosLink(self.stack.student_domain, d.get("camera_topic", f"{NS}/camera/image_raw"))
        except Exception as e:
            self.ros_error = f"ROS 2 not available: {e}"
            print(self.ros_error)
        self.args = args
        self.t_ctl = QTimer(self)
        self.t_ctl.timeout.connect(self._tick)
        self.t_ctl.start(50)        # 20 Hz control + UI
        self.t_cam = QTimer(self)
        self.t_cam.timeout.connect(self._tick_cam)
        self.t_cam.start(33)

    def start_sim(self, gazebo: str):
        if self.args.no_sim:
            return
        screens = QApplication.screens()
        show = gazebo == "on" or (gazebo == "auto" and len(screens) > 1)
        pilot = self.windowHandle().screen() if self.windowHandle() else QApplication.primaryScreen()
        for s in screens:
            if s is not pilot:
                g = s.geometry()
                self.stack.gazebo_geometry = (g.x(), g.y(), g.width(), g.height())
                break
        self.stack.start(show_gazebo=show)

    # ---------------------------------------------------------------- layout
    def _build(self):
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        h = QHBoxLayout(root)
        h.setContentsMargins(10, 10, 10, 10)
        h.setSpacing(10)
        left = QVBoxLayout()
        self.fpv = FpvView(self)
        left.addWidget(self.fpv, 1)
        bar = QHBoxLayout()
        self.b_start = button("START  (A)", "ok", 64)
        self.b_land = button("LAND  (B)", "danger", 64)
        self.b_reset = button("RESET  (X)", "warn", 64)
        self.b_mode = button("RACE  (Y)", min_h=64)
        self.b_skill = button("BEGINNER", min_h=64)
        self.b_menu = button("☰", min_h=64)
        self.b_menu.setMaximumWidth(70)
        self.b_start.clicked.connect(lambda: self.on_button("a"))
        self.b_land.clicked.connect(lambda: self.on_button("b"))
        self.b_reset.clicked.connect(lambda: self.on_button("x"))
        self.b_mode.clicked.connect(lambda: self.on_button("y"))
        self.b_skill.clicked.connect(lambda: self.on_button("back"))
        self.b_menu.clicked.connect(self._menu)
        for b in (self.b_start, self.b_land, self.b_reset, self.b_mode, self.b_skill, self.b_menu):
            bar.addWidget(b)
        left.addLayout(bar)
        h.addLayout(left, 1)

        right = QVBoxLayout()
        right.setSpacing(8)
        self.map = MiniMap(self)
        right.addWidget(self.map, 5)
        rc = card()
        rl = QVBoxLayout(rc)
        rl.setContentsMargins(14, 10, 14, 10)
        rl.setSpacing(6)
        self.l_title = label("GATE RACE", "h2")
        self.dots = GateDots(self)
        self.l_hint = label("", wrap=True)
        self.l_best = label("")
        self.l_best.setStyleSheet(f"font-size: 22px; font-weight: 800; color: {theme.WARN};")
        self.l_board = label("", "mono")
        self.l_board.setStyleSheet(f"font-family: 'DejaVu Sans Mono', monospace; font-size: 15px; color: {theme.TEXT};")
        for wdg in (self.l_title, self.dots, self.l_hint, self.l_best, self.l_board):
            rl.addWidget(wdg)
        rl.addStretch(1)
        right.addWidget(rc, 4)
        self.l_status = label("", "muted", wrap=True)
        right.addWidget(self.l_status)
        rw = QWidget()
        rw.setLayout(right)
        rw.setFixedWidth(380)
        h.addWidget(rw)

    def _menu(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Menu")
        dlg.setMinimumWidth(520)
        lay = QVBoxLayout(dlg)
        lay.setContentsMargins(20, 20, 20, 20)
        items = [("Show Gazebo chase view", lambda: self.stack.start_view()),
                 ("Restart simulation", self._restart_sim),
                 ("Rename last pilot…", self._rename_last),
                 ("Clear leaderboard", self._clear_board),
                 ("Quit (stops the simulation)", self.close),
                 ("Close menu", lambda: None)]
        for text, fn in items:
            b = button(text, min_h=60)
            b.clicked.connect(lambda _=False, f=fn: (dlg.accept(), f()))
            lay.addWidget(b)
        dlg.exec_()

    def _restart_sim(self):
        if Confirm.ask(self, "Restart simulation?", "Gazebo, ArduPilot and MAVROS are restarted (~1 minute).",
                       "Restart"):
            self.stack.stop()
            self.set_state("BOOT")
            QTimer.singleShot(1500, lambda: self.start_sim(self.args.gazebo))

    def _clear_board(self):
        if Confirm.ask(self, "Clear leaderboard?", "All times will be deleted.", "Clear", danger=True):
            self.board.clear()

    def _rename_last(self):
        if not self.board.rows:
            return
        last = max(self.board.rows, key=lambda r: r["date"])
        name = OnScreenKeyboard.get(self, last["name"])
        if name:
            last["name"] = name[:16]
            self.board.save()

    # ---------------------------------------------------------------- state machine
    def set_state(self, s: str):
        self.state = s
        self.state_t = time.monotonic()

    def show_flash(self, text: str, color: str = theme.TEXT, dur: float = 2.5):
        self.flash = (text, color, time.monotonic() + dur)

    def overlay(self):
        now = time.monotonic()
        if self.flash[0] and now < self.flash[2]:
            return self.flash[0], self.flash[1]
        s = self.state
        if s == "BOOT":
            st = self.stack.check()
            lines = ["STARTING SIMULATION"]
            for k, lbl in (("gazebo", "Gazebo"), ("sitl", "ArduPilot"), ("gate", "Safety gate + MAVROS")):
                lines.append(f"{lbl}: {st[k]}")
            if self.ros_error:
                lines.append(self.ros_error)
            return "\n".join(lines), theme.TEXT
        if s == "WAITING":
            return "Drone is warming up (GPS / EKF)…", theme.MUTED
        if s == "READY":
            hint = "PRESS  A  TO TAKE OFF"
            if self.race_mode:
                hint += "\nfly through the gates in order,\nthen land on the pad"
            return hint, theme.OK
        if s == "STARTING":
            return "TAKING OFF…", theme.WARN
        if s == "CRASHED":
            return "CRASH! resetting…", theme.BAD
        if s == "RESETTING":
            return f"Resetting the drone… {now - self.state_t:.0f}s", theme.WARN
        if s == "FINISHED":
            return f"FINISHED  {fmt_time(self.race.elapsed(now))}\n{self.flash_rank}", theme.OK
        return "", theme.TEXT

    flash_rank = ""

    def on_button(self, b: str):
        s = self.state
        if b in ("a", "start"):
            if s in ("READY", "FINISHED"):
                self._takeoff()
            elif s == "WAITING":
                self.show_flash("Drone not ready yet - wait a few seconds", theme.WARN)
        elif b == "b":
            if s in ("FLYING", "STARTING"):
                self._admin_async("land", lock=False)
                self.set_state("LANDING")
        elif b == "x":
            if s not in ("BOOT", "RESETTING"):
                self._reset()
        elif b == "y":
            if s in ("READY", "FINISHED", "WAITING"):
                self.race_mode = not self.race_mode
                self.race.reset()
                self.b_mode.setText("RACE  (Y)" if self.race_mode else "FREE  (Y)")
        elif b == "back":
            self.expert = not self.expert
            self.b_skill.setText("EXPERT" if self.expert else "BEGINNER")

    def _admin_async(self, cmd, done=None, **kw):
        def run():
            c = AdminClient(self.admin.path)
            if not c.connect():
                if done:
                    done({"ok": False, "msg": "gate not running"})
                return
            cid = c.send(cmd, **kw)
            t0 = time.monotonic()
            res = {"ok": False, "msg": "timeout"}
            while time.monotonic() - t0 < 30:
                c.poll()
                rep = [r for r in c.pop_replies() if r.get("id") == cid]
                if rep:
                    res = rep[0]
                    break
                time.sleep(0.05)
            c.close()
            if done:
                done(res)
        threading.Thread(target=run, daemon=True).start()

    def _takeoff(self):
        self.race.reset()
        self.set_state("STARTING")
        self.pilot = self.board.next_pilot_name() if self.race_mode else ""

        def done(res):
            if not res.get("ok"):
                self.show_flash("Not ready: " + str(res.get("msg", ""))[:60], theme.WARN, 4)
                self.set_state("READY")
        self._admin_async("arm_takeoff", done, alt=self.takeoff_alt)

    def _reset(self):
        if self._busy:
            return
        self._busy = True
        self.set_state("RESETTING")
        self.race.reset()

        def run():
            try:
                st = self.tel["state"]
                if st and st[1]:
                    self._admin_sync("kill")
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < 3 and self.tel["state"] and self.tel["state"][1]:
                        time.sleep(0.1)
                time.sleep(0.8)
                ok = self.ros.teleport(0.0, 0.0) if self.ros else False
                if not ok:
                    ok = self.stack.teleport_cli(0.0, 0.0)
                self._admin_sync("release_lock")
                t0 = time.monotonic()
                while time.monotonic() - t0 < 45:
                    pose = self.tel["pose"]
                    if pose and math.hypot(pose[0][0], pose[0][1]) < 1.0 and abs(pose[0][2]) < 0.6:
                        break
                    time.sleep(0.2)
                self.map.trail.clear()
                time.sleep(1.0)
            finally:
                self._busy = False
                self.set_state("WAITING")
        threading.Thread(target=run, daemon=True).start()

    def _admin_sync(self, cmd, **kw):
        ev = threading.Event()
        out = {}
        self._admin_async(cmd, lambda r: (out.update(r), ev.set()), **kw)
        ev.wait(15)
        return out

    # ---------------------------------------------------------------- periodic
    def _tick_cam(self):
        if not self.ros:
            return
        with self.ros.lock:
            img = self.ros.image
            self.ros.image = None
        if img:
            self.fpv.set_image(*img)
        self.fpv.update()

    def _tick(self):
        now = time.monotonic()
        r = self.ros
        if r:
            with r.lock:
                pose = r.pose if now - r.pose_t < 1.0 else None
                self.tel = {"pose": pose, "vel": r.vel, "state": r.state if now - r.state_t < 3 else None,
                            "battery": r.battery, "landed": r.landed,
                            "fence_warn": any(now - t < 0.6 and "slowed" in m for t, m in r.gate_errors)}
                texts = [t for ts, t in r.texts if now - ts < 6]
        else:
            texts = []
        tel = self.tel
        st = tel["state"]
        connected, armed = (st[0], st[1]) if st else (False, False)
        pose = tel["pose"]
        z = pose[0][2] if pose else 0.0
        landed_state = tel.get("landed", 0)
        on_ground = landed_state == 1 or (landed_state == 0 and z < 0.3)

        # ---- inputs
        sticks = self.pad.read() if self.pad.ok else None
        ks = self.keys.read()
        pressed = (sticks.pressed if sticks else set()) | ks.pressed
        for b in pressed:
            self.on_button(b)
        src = ks if self.keys.active or not (sticks and self.pad.connected) else sticks

        # ---- race (before the state machine so a touchdown on the pad counts)
        if pose is not None and self.race_mode and self.state in ("FLYING", "LANDING"):
            for ev in self.race.update(pose[0], on_ground, now):
                if ev.kind == "gate":
                    g = self.course.gates[ev.gate]
                    self.show_flash(f"GATE {ev.gate + 1} ✓", g.qcolor_hex, 1.2)
                elif ev.kind == "all_gates":
                    self.show_flash("ALL GATES!\nland on the pad (B)", theme.OK, 3)

        # ---- state machine
        s = self.state
        if s == "BOOT":
            if self.stack.state.get("gate") == "running" or self.args.no_sim:
                if connected:
                    self.set_state("WAITING")
        elif s == "WAITING":
            if connected and pose is not None and not armed and now - self.state_t > 2.0:
                self.set_state("READY")
        elif s == "STARTING":
            if armed and z > self.takeoff_alt - 0.5:
                self.set_state("FLYING")
                if self.race_mode:
                    self.race.start(now)
                    self.show_flash("GO!", theme.OK, 1.2)
            elif now - self.state_t > 25:
                self.set_state("READY")
        elif s in ("FLYING", "LANDING"):
            if self.race.finished:
                self._finish()
            elif not armed and on_ground:
                self.set_state("READY")
        elif s == "FINISHED":
            pass

        # ---- crash detection
        if pose is not None and s in ("FLYING", "STARTING", "LANDING"):
            roll, pitch, _ = quat_to_rpy(pose[1])
            tilted = abs(roll) > math.radians(65) or abs(pitch) > math.radians(65)
            dropped = self._last_armed and not armed and z > 0.8
            if tilted or dropped:
                self._crash_t = self._crash_t or now
                if dropped or now - self._crash_t > 0.6:
                    self.set_state("CRASHED")
                    self._crash_t = None
            else:
                self._crash_t = None
        if s == "CRASHED" and now - self.state_t > 1.8:
            self._reset()
        self._last_armed = armed

        # ---- flight commands (20 Hz)
        if self.state == "FLYING" and r is not None and armed and st[2] == "GUIDED":
            sk = self.skill.get("expert" if self.expert else "beginner", {})
            vxy = float(sk.get("xy", 6.0 if self.expert else 3.0))
            vz = float(sk.get("z", 2.5 if self.expert else 1.5))
            yr = math.radians(float(sk.get("yaw_deg", 90 if self.expert else 60)))
            boost = 1.0 + (0.5 * src.boost if self.expert else 0.0)
            fx, fy = src.pitch * vxy * boost, -src.roll * vxy * boost
            n = math.hypot(fx, fy)
            if n > vxy * boost:
                fx, fy = fx / n * vxy * boost, fy / n * vxy * boost
            r.send_body_velocity(fx, fy, src.throttle * vz, -src.yaw * yr)

        # ---- side panel
        self._refresh_panel(texts)
        self.map.update()

    def _finish(self):
        t = self.race.elapsed(time.monotonic())
        rank = self.board.add(self.pilot or "Pilot", t)
        best = self.board.rows[0]["time"] if self.board.rows else t
        self.flash_rank = ("NEW RECORD!" if rank == 1 else
                           f"#{rank} on the leaderboard" if rank else f"best: {fmt_time(best)}")
        self.set_state("FINISHED")

    def _refresh_panel(self, texts):
        self.dots.setVisible(self.race_mode)
        if self.race_mode:
            self.l_title.setText("GATE RACE")
            nxt = self.race.next_gate
            if nxt < len(self.course.gates):
                g = self.course.gates[nxt]
                self.l_hint.setText(f"Next: <b style='color:{g.qcolor_hex}'>gate {nxt + 1}</b>")
            else:
                self.l_hint.setText("<b>Land on the H pad!</b>")
        else:
            self.l_title.setText("FREE FLIGHT")
            self.l_hint.setText("Explore! The invisible fence keeps you inside the arena.")
        rows = self.board.rows[:7]
        self.l_best.setText(f"BEST  {fmt_time(rows[0]['time'])}" if rows else "")
        if rows:
            self.l_board.setText("\n".join(f"{i + 1}. {r['name'][:12]:12s} {fmt_time(r['time'])}"
                                           for i, r in enumerate(rows)))
        else:
            self.l_board.setText("No times yet - be the first!")
        pad = f"gamepad: {self.pad.name}" if self.pad.connected else \
            ("no gamepad (start from Steam or plug one in) · keyboard: WASD + arrows" if self.pad.ok else
             f"no gamepad support: {self.pad.error}")
        warn = [t for t in texts if t.startswith("PreArm") or "fail" in t.lower() or "glitch" in t.lower()]
        extra = f"\n{warn[-1]}" if warn else ""
        self.l_status.setText(f"{pad}\nskill: {'EXPERT' if self.expert else 'BEGINNER'} · state {self.state}{extra}")
        self.b_start.setEnabled(self.state in ("READY", "FINISHED"))
        self.b_land.setEnabled(self.state in ("FLYING", "STARTING"))

    # ---------------------------------------------------------------- keys / close
    _QT_KEYS = {Qt.Key_Space: "space", Qt.Key_Up: "up", Qt.Key_Down: "down", Qt.Key_Left: "left",
                Qt.Key_Right: "right", Qt.Key_Tab: "tab", Qt.Key_Return: "return", Qt.Key_Escape: "escape",
                Qt.Key_Shift: "shift"}

    def _keyname(self, e):
        if e.key() in self._QT_KEYS:
            return self._QT_KEYS[e.key()]
        return e.text().lower() if e.text() else ""

    def keyPressEvent(self, e):
        if not e.isAutoRepeat():
            self.keys.key(self._keyname(e), True)

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat():
            self.keys.key(self._keyname(e), False)

    def focusNextPrevChild(self, _next):
        return False  # keep Tab for the mode toggle

    def closeEvent(self, e):
        if not self.args.no_sim:
            self.stack.stop()
        if self.ros:
            self.ros.close()
        e.accept()


class OnScreenKeyboard(QDialog):
    def __init__(self, parent, initial=""):
        super().__init__(parent)
        self.setWindowTitle("Your name")
        self.text = initial
        lay = QVBoxLayout(self)
        self.l = label(self.text or " ", "h1")
        lay.addWidget(self.l)
        grid = QGridLayout()
        keys = "QWERTYUIOPASDFGHJKLZXCVBNM"
        for i, k in enumerate(keys):
            b = button(k, min_h=56)
            b.setMinimumWidth(56)
            b.clicked.connect(lambda _=False, ch=k: self._add(ch))
            grid.addWidget(b, i // 10, i % 10)
        lay.addLayout(grid)
        row = QHBoxLayout()
        for txt, fn in (("SPACE", lambda: self._add(" ")), ("⌫", self._back), ("OK", self.accept)):
            b = button(txt, "primary" if txt == "OK" else "", 60)
            b.clicked.connect(fn)
            row.addWidget(b)
        lay.addLayout(row)

    def _add(self, ch):
        if len(self.text) < 16:
            self.text += ch
            self.l.setText(self.text)

    def _back(self):
        self.text = self.text[:-1]
        self.l.setText(self.text or " ")

    @staticmethod
    def get(parent, initial=""):
        d = OnScreenKeyboard(parent, initial)
        return d.text.strip() if d.exec_() == QDialog.Accepted else None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--windowed", action="store_true")
    ap.add_argument("--no-sim", action="store_true", help="attach to a simulation that is already running")
    ap.add_argument("--gazebo", choices=("on", "off", "auto"), default=None)
    a, rest = ap.parse_known_args(argv)
    app = QApplication(sys.argv[:1] + rest)
    app.setStyleSheet(theme.QSS)
    w = DemoWindow(a)
    gz = a.gazebo or str((w.d.get("show_gazebo", "auto"))).lower()
    screens = QApplication.screens()
    pilot = next((s for s in screens if (s.geometry().width(), s.geometry().height()) in ((1280, 800), (800, 1280))),
                 QApplication.primaryScreen())
    w.resize(1280, 800)
    if a.windowed:
        w.show()
    else:
        w.setGeometry(pilot.geometry())
        w.showFullScreen()
    QTimer.singleShot(300, lambda: w.start_sim(gz))
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
