from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from .joint_config import JointRuntimeConfig


@dataclass(frozen=True)
class JointStateSnapshot:
    desired: dict[str, float]
    commanded: dict[str, float]
    observed: dict[str, float]
    filtered: dict[str, float]
    controller_intent: dict[str, float]
    controller_source: str
    controller_updated_at: float | None


class JointStateStore:
    def __init__(self, joint_configs: dict[str, JointRuntimeConfig]) -> None:
        self._joint_configs = joint_configs
        self._lock = threading.RLock()
        defaults = {name: cfg.default for name, cfg in joint_configs.items()}
        self._desired = dict(defaults)
        self._commanded = dict(defaults)
        self._observed = dict(defaults)
        self._filtered = dict(defaults)
        self._controller_intent = {name: 0.0 for name in joint_configs}
        self._controller_source = "system"
        self._controller_updated_at: float | None = None

    def set_desired(self, targets: dict[str, float]) -> dict[str, float]:
        with self._lock:
            for name, value in targets.items():
                if name not in self._joint_configs:
                    continue
                cfg = self._joint_configs[name]
                self._desired[name] = max(cfg.minimum, min(cfg.maximum, float(value)))
            return dict(self._desired)

    def reset_desired(self, targets: dict[str, float]) -> dict[str, float]:
        with self._lock:
            self._desired = {}
            for name, cfg in self._joint_configs.items():
                value = float(targets.get(name, cfg.default))
                self._desired[name] = max(cfg.minimum, min(cfg.maximum, value))
            return dict(self._desired)

    def set_commanded(self, targets: dict[str, float]) -> None:
        with self._lock:
            for name, value in targets.items():
                if name in self._joint_configs:
                    self._commanded[name] = float(value)

    def set_observed(self, joints: dict[str, float]) -> None:
        with self._lock:
            for name, value in joints.items():
                if name in self._joint_configs:
                    self._observed[name] = float(value)

    def set_filtered(self, joints: dict[str, float]) -> None:
        with self._lock:
            for name, value in joints.items():
                if name in self._joint_configs:
                    self._filtered[name] = float(value)

    def set_controller_intent(self, intents: dict[str, float], source: str) -> dict[str, float]:
        with self._lock:
            self._controller_intent = {
                name: max(-1.0, min(1.0, float(intents.get(name, 0.0))))
                for name in self._joint_configs
            }
            self._controller_source = source
            self._controller_updated_at = time.time()
            return dict(self._controller_intent)

    def snapshot(self) -> JointStateSnapshot:
        with self._lock:
            return JointStateSnapshot(
                desired=dict(self._desired),
                commanded=dict(self._commanded),
                observed=dict(self._observed),
                filtered=dict(self._filtered),
                controller_intent=dict(self._controller_intent),
                controller_source=self._controller_source,
                controller_updated_at=self._controller_updated_at,
            )
