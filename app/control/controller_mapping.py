from __future__ import annotations

from dataclasses import dataclass

from .joint_config import ControllerBinding, JointRuntimeConfig


@dataclass(frozen=True)
class PS4Snapshot:
    axes: dict[str, float]
    buttons: dict[str, float]


def apply_deadzone(value: float, deadzone: float) -> float:
    value = float(value)
    deadzone = max(0.0, min(0.95, float(deadzone)))
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return scaled * (1 if value > 0 else -1)


def apply_curve(value: float, curve: float) -> float:
    value = float(value)
    magnitude = abs(value)
    if magnitude == 0:
        return 0.0
    exponent = max(0.1, float(curve))
    return (magnitude**exponent) * (1 if value > 0 else -1)


class PS4ControllerMapper:
    def __init__(self, joint_configs: dict[str, JointRuntimeConfig]) -> None:
        self._joint_configs = joint_configs

    def map_snapshot(self, snapshot: PS4Snapshot) -> dict[str, float]:
        intents: dict[str, float] = {}
        for name, cfg in self._joint_configs.items():
            binding = cfg.controller_binding
            if not binding or not binding.enabled:
                continue
            value = self._resolve_binding(binding, snapshot)
            if abs(value) < 1e-4:
                continue
            intents[name] = max(-1.0, min(1.0, value))
        return intents

    def _resolve_binding(self, binding: ControllerBinding, snapshot: PS4Snapshot) -> float:
        value = 0.0
        if binding.kind == "axis" and binding.axis:
            value = float(snapshot.axes.get(binding.axis, 0.0))
        elif binding.kind == "trigger_pair":
            negative = float(snapshot.buttons.get(binding.negative_trigger or "", 0.0))
            positive = float(snapshot.buttons.get(binding.positive_trigger or "", 0.0))
            value = positive - negative
        elif binding.kind == "button_pair":
            negative = float(snapshot.buttons.get(binding.negative_button or "", 0.0))
            positive = float(snapshot.buttons.get(binding.positive_button or "", 0.0))
            value = positive - negative

        value = apply_deadzone(value, binding.deadzone)
        value = apply_curve(value, binding.curve)
        value *= binding.scale * binding.sign
        return max(-1.0, min(1.0, value))
