from __future__ import annotations

from .joint_config import JointRuntimeConfig


class MotionFilter:
    def __init__(self, joint_configs: dict[str, JointRuntimeConfig]) -> None:
        self._joint_configs = joint_configs

    def step(
        self,
        desired: dict[str, float],
        commanded: dict[str, float],
        dt: float,
    ) -> dict[str, float]:
        filtered: dict[str, float] = {}
        for name, cfg in self._joint_configs.items():
            desired_value = float(desired.get(name, cfg.default))
            desired_value = max(cfg.minimum, min(cfg.maximum, desired_value))
            current_value = float(commanded.get(name, desired_value))

            stepped = desired_value
            if cfg.max_delta_per_command is not None:
                delta = stepped - current_value
                limit = float(cfg.max_delta_per_command)
                if abs(delta) > limit:
                    stepped = current_value + limit * (1 if delta > 0 else -1)

            if cfg.max_degrees_per_second is not None:
                delta = stepped - current_value
                max_rate_delta = max(0.05, float(cfg.max_degrees_per_second) * max(dt, 0.01))
                if abs(delta) > max_rate_delta:
                    stepped = current_value + max_rate_delta * (1 if delta > 0 else -1)

            filtered[name] = max(cfg.minimum, min(cfg.maximum, stepped))
        return filtered
