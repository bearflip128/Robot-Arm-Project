"""Wired USB gamepad reader.

Feeds the controller directly in-process — no HTTP hop, so a laptop-attached
pad has no network latency in the loop at all.
"""

from __future__ import annotations

import importlib
import logging
import os
import threading
import time

from .controller import Controller
from .inputs import PadSnapshot
from .joints import clamp

log = logging.getLogger("armctl.pad")

# Defaults for a DualShock 4 under SDL's GameController mapping.
#
# These indices are NOT universal — they differ between controllers, between
# SDL versions, and between SDL's raw-joystick and GameController mappings. On
# this pad L1/R1 are buttons 9 and 10; the older raw layout puts them at 4 and 5,
# which on the GameController layout are Share and PS. Reading the wrong pair
# looks exactly like a dead gripper: everything else works, those buttons do
# nothing. Override in config under `gamepad:` if yours differs, and use
# tools/pad_probe.py to find the real indices.
DEFAULT_AXES = {"left_x": 0, "left_y": 1, "right_x": 2, "right_y": 3}
DEFAULT_TRIGGER_AXES = {"l2": 4, "r2": 5}
DEFAULT_BUTTONS = {"l1": 9, "r1": 10}


class PadService:
    def __init__(self, controller: Controller, *, poll_hz: float = 90.0,
                 source: str = "wired_pad", layout: dict | None = None) -> None:
        layout = layout or {}
        self._axes_map = {**DEFAULT_AXES, **(layout.get("axes") or {})}
        self._trigger_map = {**DEFAULT_TRIGGER_AXES, **(layout.get("trigger_axes") or {})}
        self._button_map = {**DEFAULT_BUTTONS, **(layout.get("buttons") or {})}
        self._controller = controller
        self._interval = 1.0 / max(20.0, poll_hz)
        self._source = source
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._pygame = None
        self._pad = None
        self._was_active = False
        self._last_axes: dict[str, float] = {}
        self._last_buttons: dict[str, float] = {}
        self.device: str | None = None
        self.error: str | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="armctl-pad")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def status(self) -> dict:
        return {
            "connected": self._pad is not None,
            "device": self.device,
            "error": self.error,
            "active": self._was_active,
            "axes": dict(self._last_axes),
            "buttons": dict(self._last_buttons),
        }

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._poll()
            except Exception as exc:
                if self.error != str(exc):
                    log.warning("Gamepad unavailable: %s", exc)
                self.error = str(exc)
                self._release()
            self._stop.wait(self._interval)

    def _poll(self) -> None:
        pad = self._acquire()
        if pad is None:
            self._release_intent()
            return

        axes = {name: _axis(pad, index) for name, index in self._axes_map.items()}
        buttons = {name: _button(pad, index) for name, index in self._button_map.items()}
        for name, index in self._trigger_map.items():
            buttons[name] = _trigger(pad, index)

        snapshot = PadSnapshot(axes=axes, buttons=buttons)
        self._last_axes, self._last_buttons = axes, buttons
        # Keep publishing for one beat after release so the control loop sees an
        # explicit zero rather than only a watchdog timeout.
        if snapshot.active or self._was_active:
            self._controller.set_pad(snapshot, self._source)
        self._was_active = snapshot.active

    def _release_intent(self) -> None:
        if self._was_active:
            self._controller.set_pad(PadSnapshot(), self._source)
            self._was_active = False

    def _acquire(self):
        """Get the pad, holding on to it once found.

        Deliberately does NOT re-check joystick.get_count() on every poll. This
        process has no window, and without one SDL's device count on Windows
        intermittently reads 0 even while the pad is plugged in and reporting —
        which silently dropped a working controller seconds after connecting
        it. Removal is instead detected from JOYDEVICEREMOVED and from the
        joystick failing to read, both of which are actually trustworthy.
        """
        if self._pygame is None:
            os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            self._pygame = importlib.import_module("pygame")
            self._pygame.init()
        pygame = self._pygame
        if not pygame.joystick.get_init():
            pygame.joystick.init()

        # Draining the queue also pumps it, which is what refreshes axis state.
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                log.info("Gamepad disconnected.")
                self._release()

        if self._pad is not None:
            try:
                if self._pad.get_init():
                    return self._pad
            except Exception:
                pass
            self._release()

        if pygame.joystick.get_count() <= 0:
            return None

        pad = pygame.joystick.Joystick(0)
        pad.init()
        self._pad = pad
        self.device = str(pad.get_name() or "Gamepad")
        self.error = None
        log.info("Gamepad connected: %s", self.device)
        return pad

    def _release(self) -> None:
        if self._pad is not None:
            try:
                self._pad.quit()
            except Exception:
                pass
        self._pad = None
        self.device = None


def _axis(pad, index: int) -> float:
    if index >= pad.get_numaxes():
        return 0.0
    value = float(pad.get_axis(index))
    return 0.0 if value != value else clamp(value, -1.0, 1.0)


def _button(pad, index: int) -> float:
    if index >= pad.get_numbuttons():
        return 0.0
    return 1.0 if pad.get_button(index) else 0.0


def _trigger(pad, index: int) -> float:
    """SDL reports triggers as an axis resting at -1 and pressing to +1.

    An exact 0.0 means the driver has not reported this axis yet; mapping that
    through would read as a permanent half-press and pan the base forever.
    """
    if index >= pad.get_numaxes():
        return 0.0
    value = float(pad.get_axis(index))
    if value != value or abs(value) < 1e-6:
        return 0.0
    return clamp((value + 1.0) / 2.0, 0.0, 1.0)
