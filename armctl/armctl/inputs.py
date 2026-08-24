"""Gamepad snapshot -> normalized velocity intent, in -1..1 per joint.

Pure functions only. The same code serves the wired pygame reader and the
browser Gamepad API, so both paths feel identical.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .joints import Binding, JointSpec, clamp

AXES = ("left_x", "left_y", "right_x", "right_y")
BUTTONS = ("l1", "r1", "l2", "r2")


@dataclass(frozen=True)
class PadSnapshot:
    axes: dict[str, float] = field(default_factory=dict)
    buttons: dict[str, float] = field(default_factory=dict)

    @classmethod
    def parse(cls, payload: dict) -> PadSnapshot:
        raw_axes = payload.get("axes") or {}
        raw_buttons = payload.get("buttons") or {}
        return cls(
            axes={name: _finite(raw_axes.get(name)) for name in AXES},
            buttons={name: _finite(raw_buttons.get(name)) for name in BUTTONS},
        )

    @property
    def active(self) -> bool:
        return any(abs(v) > 1e-3 for v in (*self.axes.values(), *self.buttons.values()))


def _finite(value) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return 0.0 if not math.isfinite(number) else clamp(number, -1.0, 1.0)


def shape(value: float, binding: Binding) -> float:
    """Deadzone, rescale, expo, sign — in that order."""
    deadzone = clamp(binding.deadzone, 0.0, 0.95)
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    # Rescale so motion starts at zero right at the deadzone edge rather than
    # jumping to `deadzone` worth of speed the instant the stick crosses it.
    magnitude = (magnitude - deadzone) / (1.0 - deadzone)
    magnitude = magnitude ** max(0.1, binding.expo)
    return clamp(math.copysign(magnitude, value) * binding.sign, -1.0, 1.0)


def resolve(binding: Binding, pad: PadSnapshot) -> float:
    if binding.kind == "axis" and binding.axis:
        raw = pad.axes.get(binding.axis, 0.0)
    elif binding.kind in ("trigger_pair", "button_pair"):
        source = pad.buttons
        positive = abs(source.get(binding.positive or "", 0.0))
        negative = abs(source.get(binding.negative or "", 0.0))
        raw = positive - negative
    else:
        return 0.0
    return shape(raw, binding)


def intent_from_pad(pad: PadSnapshot, specs: dict[str, JointSpec]) -> dict[str, float]:
    """Normalized -1..1 velocity intent for every joint that has a binding."""
    return {name: resolve(spec.binding, pad) for name, spec in specs.items()}
