"""Gamepad input via SDL (pygame) with a keyboard fallback.

Works with the Steam Deck's own controls when the Demo app is started from Steam
(Steam Input then exposes them as an Xbox pad), and with any USB/Bluetooth pad.
Mode 2 layout: left stick = climb/descend + yaw, right stick = forward/back + left/right.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, Set

BUTTONS = ("a", "b", "x", "y", "back", "start", "lb", "rb", "guide")


@dataclass
class Sticks:
    throttle: float = 0.0   # +1 climb
    yaw: float = 0.0        # +1 turn right
    pitch: float = 0.0      # +1 forward
    roll: float = 0.0       # +1 right
    boost: float = 0.0      # right trigger 0..1
    pressed: Set[str] = field(default_factory=set)   # buttons that went down since the last read
    source: str = "none"


def shape(v: float, dead: float = 0.12, expo: float = 0.35) -> float:
    if abs(v) < dead:
        return 0.0
    s = (abs(v) - dead) / (1.0 - dead)
    s = (1 - expo) * s + expo * s ** 3
    return max(-1.0, min(1.0, s if v > 0 else -s))


class Gamepad:
    def __init__(self):
        self.ok = False
        self.name = ""
        self.error = ""
        self._pg = None
        self._ctrl = None
        self._joy = None
        self._prev: Dict[str, bool] = {}
        self._next_scan = 0.0
        try:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            # containers usually have no udev daemon: let SDL watch /dev/input itself
            if not os.path.exists("/run/udev/control"):
                os.environ.setdefault("SDL_JOYSTICK_DISABLE_UDEV", "1")
            os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
            import pygame
            pygame.display.init()
            pygame.joystick.init()
            self._pg = pygame
            try:
                from pygame._sdl2 import controller
                controller.init()
                self._sdl_ctrl = controller
            except Exception:
                self._sdl_ctrl = None
            self.ok = True
        except Exception as e:
            self.error = f"pygame/SDL not available: {e}"

    def _scan(self):
        pg = self._pg
        now = time.monotonic()
        if now < self._next_scan:
            return
        self._next_scan = now + 2.0
        if pg.joystick.get_count() == 0:
            # SDL only rescans on hotplug events; re-init to pick up pads that appeared later
            pg.joystick.quit()
            pg.joystick.init()
        if self._ctrl or self._joy:
            return
        for i in range(pg.joystick.get_count()):
            if self._sdl_ctrl is not None and self._sdl_ctrl.is_controller(i):
                self._ctrl = self._sdl_ctrl.Controller(i)
                self.name = self._ctrl.name or "game controller"
                return
        if pg.joystick.get_count() > 0:
            self._joy = pg.joystick.Joystick(0)
            self._joy.init()
            self.name = self._joy.get_name()

    @property
    def connected(self) -> bool:
        return bool(self._ctrl or self._joy)

    def read(self) -> Sticks:
        st = Sticks()
        if not self.ok:
            return st
        pg = self._pg
        try:
            pg.event.pump()
            for e in pg.event.get():
                if e.type in (getattr(pg, "JOYDEVICEREMOVED", -1), getattr(pg, "CONTROLLERDEVICEREMOVED", -2)):
                    self._ctrl = self._joy = None
                    self.name = ""
            self._scan()
            buttons: Dict[str, bool] = {}
            if self._ctrl is not None:
                c = self._ctrl
                ax = lambda a: c.get_axis(a) / 32767.0  # noqa: E731
                st.throttle = -shape(ax(pg.CONTROLLER_AXIS_LEFTY))
                st.yaw = shape(ax(pg.CONTROLLER_AXIS_LEFTX))
                st.pitch = -shape(ax(pg.CONTROLLER_AXIS_RIGHTY))
                st.roll = shape(ax(pg.CONTROLLER_AXIS_RIGHTX))
                st.boost = max(0.0, ax(pg.CONTROLLER_AXIS_TRIGGERRIGHT))
                m = {"a": pg.CONTROLLER_BUTTON_A, "b": pg.CONTROLLER_BUTTON_B, "x": pg.CONTROLLER_BUTTON_X,
                     "y": pg.CONTROLLER_BUTTON_Y, "back": pg.CONTROLLER_BUTTON_BACK,
                     "start": pg.CONTROLLER_BUTTON_START, "lb": pg.CONTROLLER_BUTTON_LEFTSHOULDER,
                     "rb": pg.CONTROLLER_BUTTON_RIGHTSHOULDER, "guide": pg.CONTROLLER_BUTTON_GUIDE}
                buttons = {k: bool(c.get_button(v)) for k, v in m.items()}
                st.source = self.name
            elif self._joy is not None:
                j = self._joy
                n = j.get_numaxes()
                g = lambda i: j.get_axis(i) if i < n else 0.0  # noqa: E731
                # xpad layout: 0 LX, 1 LY, 2 LT, 3 RX, 4 RY, 5 RT
                st.throttle, st.yaw = -shape(g(1)), shape(g(0))
                st.pitch, st.roll = -shape(g(4)), shape(g(3))
                st.boost = max(0.0, (g(5) + 1) / 2) if n > 5 else 0.0
                nb = j.get_numbuttons()
                order = ("a", "b", "x", "y", "lb", "rb", "back", "start", "guide")
                buttons = {k: bool(j.get_button(i)) if i < nb else False for i, k in enumerate(order)}
                st.source = self.name
        except Exception as e:  # a pad unplugged mid-read etc.
            self.error = str(e)
            self._ctrl = self._joy = None
            return st
        for k, v in buttons.items():
            if v and not self._prev.get(k):
                st.pressed.add(k)
        self._prev = buttons
        return st


class KeyboardSticks:
    """WASD = move, arrows = climb/turn, Space = A (take off), L = B (land), R = X (reset), Tab = Y."""

    KEYMAP = {"space": "a", "l": "b", "r": "x", "tab": "y", "return": "start", "escape": "back"}

    def __init__(self):
        self.down: Set[str] = set()
        self.pressed: Set[str] = set()

    def key(self, name: str, is_down: bool):
        name = name.lower()
        if is_down:
            if name not in self.down and name in self.KEYMAP:
                self.pressed.add(self.KEYMAP[name])
            self.down.add(name)
        else:
            self.down.discard(name)

    def read(self) -> Sticks:
        d = self.down
        st = Sticks(source="keyboard")
        st.pitch = (1.0 if "w" in d else 0.0) - (1.0 if "s" in d else 0.0)
        st.roll = (1.0 if "d" in d else 0.0) - (1.0 if "a" in d else 0.0)
        st.throttle = (1.0 if "up" in d else 0.0) - (1.0 if "down" in d else 0.0)
        st.yaw = (1.0 if "right" in d else 0.0) - (1.0 if "left" in d else 0.0)
        st.boost = 1.0 if "shift" in d else 0.0
        st.pressed, self.pressed = self.pressed, set()
        return st

    @property
    def active(self) -> bool:
        return bool(self.down)
