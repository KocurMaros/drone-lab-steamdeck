"""Reusable touch-friendly widgets."""
from __future__ import annotations

import collections
import math
import time
from typing import List, Optional, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import (QDialog, QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget)

from . import theme


def label(text: str, obj: str = "", wrap: bool = False) -> QLabel:
    lb = QLabel(text)
    if obj:
        lb.setObjectName(obj)
    lb.setWordWrap(wrap)
    return lb


def button(text: str, obj: str = "", min_h: int = 52) -> QPushButton:
    b = QPushButton(text)
    if obj:
        b.setObjectName(obj)
    b.setMinimumHeight(min_h)
    b.setFocusPolicy(Qt.NoFocus)
    return b


def card() -> QFrame:
    f = QFrame()
    f.setObjectName("card")
    return f


class Tile(QFrame):
    """Small status tile: caption + big value, colored by state."""

    def __init__(self, caption: str):
        super().__init__()
        self.setObjectName("card")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(2)
        self.cap = label(caption, "muted")
        self.val = QLabel("-")
        self.val.setStyleSheet("font-size: 22px; font-weight: 800;")
        lay.addWidget(self.cap)
        lay.addWidget(self.val)
        self.setMinimumHeight(70)

    def set(self, text: str, color: str = theme.TEXT):
        self.val.setText(text)
        self.val.setStyleSheet(f"font-size: 22px; font-weight: 800; color: {color};")


class HoldButton(QPushButton):
    """Fires `held` only after being pressed continuously for `hold_s` seconds."""

    held = pyqtSignal()

    def __init__(self, text: str, hold_s: float = 1.5, obj: str = "danger"):
        super().__init__(text)
        self.setObjectName(obj)
        self.setMinimumHeight(52)
        self.setFocusPolicy(Qt.NoFocus)
        self.hold_s = hold_s
        self._t0: Optional[float] = None
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._base = text
        self.pressed.connect(self._start)
        self.released.connect(self._cancel)

    def _start(self):
        self._t0 = time.monotonic()
        self._timer.start(50)

    def _cancel(self):
        self._t0 = None
        self._timer.stop()
        self.setText(self._base)

    def _tick(self):
        if self._t0 is None:
            return
        frac = (time.monotonic() - self._t0) / self.hold_s
        if frac >= 1.0:
            self._cancel()
            self.held.emit()
        else:
            self.setText(f"{self._base}  {'█' * int(frac * 8)}{'░' * (8 - int(frac * 8))}")


class Confirm(QDialog):
    """Big touch-friendly yes/no dialog."""

    def __init__(self, parent, title: str, text: str, yes: str = "Yes", no: str = "Cancel", danger=False):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(True)
        self.setMinimumWidth(620)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 24, 24, 24)
        lay.setSpacing(18)
        lay.addWidget(label(title, "h2"))
        lay.addWidget(label(text, wrap=True))
        row = QHBoxLayout()
        b_no = button(no, min_h=64)
        b_yes = button(yes, "danger" if danger else "primary", min_h=64)
        b_no.clicked.connect(self.reject)
        b_yes.clicked.connect(self.accept)
        row.addWidget(b_no)
        row.addWidget(b_yes)
        lay.addLayout(row)

    @staticmethod
    def ask(parent, title, text, yes="Yes", no="Cancel", danger=False) -> bool:
        return Confirm(parent, title, text, yes, no, danger).exec_() == QDialog.Accepted


class FenceMap(QWidget):
    """Top-down view of the fence with the drone, its heading and trail. Units: fence coordinates."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fence: Optional[dict] = None
        self.pos: Optional[Tuple[float, float, float]] = None
        self.yaw: Optional[float] = None
        self.locked = False
        self.trail: collections.deque = collections.deque(maxlen=300)
        self.extra_points: List[Tuple[float, float, str]] = []

    def update_state(self, fence: dict, pos, yaw, locked: bool):
        self.fence = fence
        self.locked = locked
        if pos:
            p = (pos[0], pos[1], pos[2])
            if not self.trail or math.dist(self.trail[-1][:2], p[:2]) > 0.05:
                self.trail.append(p)
        self.pos = tuple(pos) if pos else None
        self.yaw = yaw
        self.update()

    def _outline(self) -> List[Tuple[float, float]]:
        f = self.fence or {}
        if f.get("type") == "box":
            (x0, x1), (y0, y1) = f["x"], f["y"]
            return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if f.get("type") == "polygon":
            return [tuple(p) for p in f.get("plane_points", [])]
        if f.get("type") == "circle":
            r = f["radius"]
            return [(r * math.cos(a / 36 * 2 * math.pi), r * math.sin(a / 36 * 2 * math.pi)) for a in range(36)]
        return []

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(theme.PANEL))
        pts = self._outline()
        if not pts:
            p.setPen(QColor(theme.MUTED))
            p.drawText(self.rect(), Qt.AlignCenter, "no fence")
            return
        m = float((self.fence or {}).get("margin", 0.0))
        xs = [a for a, _ in pts] + ([self.pos[0]] if self.pos else [])
        ys = [b for _, b in pts] + ([self.pos[1]] if self.pos else [])
        minx, maxx, miny, maxy = min(xs) - m - 0.5, max(xs) + m + 0.5, min(ys) - m - 0.5, max(ys) + m + 0.5
        w, h = self.width() - 20, self.height() - 34
        s = min(w / (maxx - minx), h / (maxy - miny))
        ox = 10 + (w - s * (maxx - minx)) / 2
        oy = 10 + (h - s * (maxy - miny)) / 2

        def T(x, y) -> QPointF:  # north/y up
            return QPointF(ox + (x - minx) * s, oy + (maxy - y) * s)

        # grid every 1 m (or 10 m for big fences)
        step = 1.0 if (maxx - minx) < 40 else 10.0
        p.setPen(QPen(QColor("#223041"), 1))
        gx = math.floor(minx / step) * step
        while gx <= maxx:
            p.drawLine(T(gx, miny), T(gx, maxy))
            gx += step
        gy = math.floor(miny / step) * step
        while gy <= maxy:
            p.drawLine(T(minx, gy), T(maxx, gy))
            gy += step
        # origin axes
        p.setPen(QPen(QColor("#3a5068"), 1.5))
        p.drawLine(T(0, 0), T(step, 0))
        p.drawLine(T(0, 0), T(0, step))
        # fence (fill + outline)
        poly = QPolygonF([T(x, y) for x, y in pts])
        col = QColor(theme.BAD if self.locked else theme.OK)
        fill = QColor(col)
        fill.setAlpha(28)
        p.setBrush(QBrush(fill))
        p.setPen(QPen(col, 2.5))
        p.drawPolygon(poly)
        p.setBrush(Qt.NoBrush)
        # trail
        if len(self.trail) > 1:
            path = QPainterPath(T(self.trail[0][0], self.trail[0][1]))
            for x, y, _ in list(self.trail)[1:]:
                path.lineTo(T(x, y))
            p.setPen(QPen(QColor(theme.ACCENT), 2, Qt.SolidLine, Qt.RoundCap))
            p.drawPath(path)
        for x, y, c in self.extra_points:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(c))
            p.drawEllipse(T(x, y), 5, 5)
        # drone
        if self.pos:
            c = T(self.pos[0], self.pos[1])
            yaw = self.yaw or 0.0
            L = 16
            tip = QPointF(c.x() + L * math.cos(yaw), c.y() - L * math.sin(yaw))
            left = QPointF(c.x() + 0.6 * L * math.cos(yaw + 2.5), c.y() - 0.6 * L * math.sin(yaw + 2.5))
            right = QPointF(c.x() + 0.6 * L * math.cos(yaw - 2.5), c.y() - 0.6 * L * math.sin(yaw - 2.5))
            p.setBrush(QColor(theme.WARN))
            p.setPen(QPen(QColor("#000000"), 1.5))
            p.drawPolygon(QPolygonF([tip, left, c, right]))
        # scale text
        p.setPen(QColor(theme.MUTED))
        f = QFont()
        f.setPointSize(10)
        p.setFont(f)
        txt = f"grid {step:g} m"
        if self.pos:
            txt += f"   x {self.pos[0]:.2f}  y {self.pos[1]:.2f}  z {self.pos[2]:.2f}"
        p.drawText(QRectF(10, self.height() - 22, self.width() - 20, 20), Qt.AlignLeft, txt)
