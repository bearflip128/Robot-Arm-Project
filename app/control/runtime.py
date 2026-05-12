from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

from app.robot_arm import JointDefinition, RobotArmAdapter, default_joint_positions

from .joint_config import JointRuntimeConfig
from .motion_filter import MotionFilter
from .state_store import JointStateStore


@dataclass(frozen=True)
class SafetyStateSnapshot:
    armed: bool
    estopped: bool
    lock_reason: str

    @property
    def motion_allowed(self) -> bool:
        return self.armed and not self.estopped


class RobotControlRuntime:
    def __init__(
        self,
        *,
        adapter: RobotArmAdapter,
        joint_definitions: dict[str, JointDefinition],
        joint_configs: dict[str, JointRuntimeConfig],
        state_store: JointStateStore,
        motion_filter: MotionFilter,
        config: dict[str, Any],
        loop_hz: float = 40.0,
    ) -> None:
        self._adapter = adapter
        self._joint_definitions = joint_definitions
        self._joint_configs = joint_configs
        self._state_store = state_store
        self._motion_filter = motion_filter
        self._config = config
        self._teleop_config = config.get("robot", {}).get("teleop", {})
        self._default_controller_max_rate = float(
            self._teleop_config.get("default_controller_max_degrees_per_second", 120.0)
        )
        self._controller_intent_timeout_s = float(self._teleop_config.get("intent_timeout_s", 0.3))
        self._controller_intent_epsilon = 0.01
        self._control_epsilon = 0.2
        self._loop_hz = float(loop_hz)
        self._loop_dt = 1.0 / self._loop_hz

        self._lock = threading.RLock()
        self._joint_sources = {name: "system" for name in joint_definitions}
        self._armed = False
        self._estopped = False
        self._lock_reason = "Controls start disarmed for remote-safe operation."
        self._last_loop_error: str | None = None
        self._last_loop_at: float | None = None
        self._loop_stop = threading.Event()
        self._loop_thread: threading.Thread | None = None

    def start(self) -> None:
        with self._lock:
            if self._loop_thread and self._loop_thread.is_alive():
                return
            self._loop_stop.clear()
            self._loop_thread = threading.Thread(
                target=self._run_loop,
                daemon=True,
                name="robot-control-loop",
            )
            self._loop_thread.start()

    def stop(self, timeout_s: float = 1.0) -> None:
        with self._lock:
            self._loop_stop.set()
            thread = self._loop_thread
        if thread:
            thread.join(timeout=timeout_s)

    @property
    def loop_hz(self) -> float:
        return self._loop_hz

    def clamp_targets(self, raw_targets: dict[str, float]) -> dict[str, float]:
        clamped: dict[str, float] = {}
        for name, value in raw_targets.items():
            definition = self._joint_definitions.get(name)
            if definition is None:
                continue
            numeric_value = float(value)
            clamped[name] = max(definition.minimum, min(definition.maximum, numeric_value))
        return clamped

    def desired_targets_snapshot(self) -> dict[str, float]:
        return self._state_store.snapshot().desired

    def set_desired_targets(self, targets: dict[str, float], source: str = "system") -> dict[str, float]:
        clamped = self.clamp_targets(targets)
        if not clamped:
            return self.desired_targets_snapshot()
        with self._lock:
            for name, value in clamped.items():
                self._joint_sources[name] = source
        return self._state_store.set_desired(clamped)

    def reset_desired_targets(
        self,
        targets: dict[str, float] | None = None,
        source: str = "system",
    ) -> dict[str, float]:
        snapshot = self.clamp_targets(targets or default_joint_positions(self._joint_definitions))
        with self._lock:
            self._joint_sources = {name: source for name in self._joint_definitions}
        return self._state_store.reset_desired(snapshot)

    def sync_from_state(self, state: dict[str, Any], source: str = "system") -> dict[str, float]:
        joints = self.clamp_targets(state.get("joints", {}))
        commanded = self.clamp_targets(state.get("commanded_joints", joints))
        if not joints:
            joints = default_joint_positions(self._joint_definitions)
        if not commanded:
            commanded = dict(joints)
        self.reset_desired_targets(joints, source=source)
        self._state_store.set_observed(joints)
        self._state_store.set_commanded(commanded)
        self._state_store.set_filtered(commanded)
        return dict(joints)

    def set_controller_intent(self, intents: dict[str, float], source: str = "controller") -> dict[str, float]:
        normalized = {
            name: max(-1.0, min(1.0, float(intents.get(name, 0.0))))
            for name in self._joint_definitions
        }
        return self._state_store.set_controller_intent(normalized, source)

    def clear_controller_intent(self, source: str = "system") -> dict[str, float]:
        return self.set_controller_intent({}, source=source)

    def controller_runtime_snapshot(self) -> dict[str, Any]:
        snapshot = self._state_store.snapshot()
        age_s = None
        if snapshot.controller_updated_at is not None:
            age_s = max(0.0, time.time() - float(snapshot.controller_updated_at))
        return {
            "intent": dict(snapshot.controller_intent),
            "source": snapshot.controller_source,
            "last_controller_at": snapshot.controller_updated_at,
            "intent_age_s": None if age_s is None else round(age_s, 3),
            "intent_active": bool(
                age_s is not None
                and age_s <= self._controller_intent_timeout_s
                and any(
                    abs(float(value)) >= self._controller_intent_epsilon
                    for value in snapshot.controller_intent.values()
                )
            ),
        }

    def current_joint_source(self, name: str) -> str:
        with self._lock:
            return str(self._joint_sources.get(name, "system"))

    def safety_snapshot(self) -> SafetyStateSnapshot:
        with self._lock:
            return SafetyStateSnapshot(
                armed=self._armed,
                estopped=self._estopped,
                lock_reason=self._lock_reason,
            )

    def motion_allowed(self) -> bool:
        return self.safety_snapshot().motion_allowed

    def arm(self, source: str = "arm") -> None:
        self.sync_from_state(self.safe_adapter_state(), source=source)
        self.clear_controller_intent(source=source)
        with self._lock:
            self._armed = True
            self._lock_reason = "System armed. Live motion commands are enabled."

    def disarm(self, source: str = "disarm", reason: str | None = None) -> None:
        self.sync_from_state(self.safe_adapter_state(), source=source)
        self.clear_controller_intent(source=source)
        with self._lock:
            self._armed = False
            self._lock_reason = reason or "System disarmed. Motion commands are blocked."

    def emergency_stop(self, source: str = "estop") -> None:
        self.sync_from_state(self.safe_adapter_state(), source=source)
        self.clear_controller_intent(source=source)
        with self._lock:
            self._estopped = True
            self._armed = False
            self._lock_reason = "Emergency stop is active. Reset before any motion."

    def reset_estop(self, source: str = "reset") -> None:
        self.sync_from_state(self.safe_adapter_state(), source=source)
        self.clear_controller_intent(source=source)
        with self._lock:
            self._estopped = False
            self._armed = False
            self._lock_reason = "Emergency stop cleared. Re-arm when ready."

    def set_lock_reason(self, reason: str) -> None:
        with self._lock:
            self._lock_reason = reason

    def safe_adapter_state(self) -> dict[str, Any]:
        try:
            return self._adapter.get_state()
        except Exception as exc:
            defaults = default_joint_positions(self._joint_definitions)
            return {
                "connected": False,
                "joints": defaults,
                "commanded_joints": defaults,
                "hardware_error": str(exc),
            }

    def loop_runtime_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "loop_hz": self._loop_hz,
                "last_loop_error": self._last_loop_error,
                "last_loop_at": self._last_loop_at,
                "controller": self.controller_runtime_snapshot(),
            }

    def _apply_controller_intent_to_desired(self, dt: float) -> dict[str, float]:
        snapshot = self._state_store.snapshot()
        desired = dict(snapshot.desired)

        if snapshot.controller_updated_at is None:
            return desired

        age_s = max(0.0, time.time() - float(snapshot.controller_updated_at))
        if age_s > self._controller_intent_timeout_s:
            if any(abs(float(value)) >= self._controller_intent_epsilon for value in snapshot.controller_intent.values()):
                self._state_store.set_controller_intent({}, snapshot.controller_source)
            return desired

        source = str(snapshot.controller_source or "controller")
        updated: dict[str, float] = {}
        for joint_name, input_value in snapshot.controller_intent.items():
            signed_input = float(input_value)
            if abs(signed_input) < self._controller_intent_epsilon:
                continue
            cfg = self._joint_configs[joint_name]
            max_rate = float(
                cfg.controller_max_degrees_per_second
                or cfg.max_degrees_per_second
                or self._default_controller_max_rate
            )
            updated[joint_name] = float(desired.get(joint_name, cfg.default)) + signed_input * max_rate * max(dt, 0.01)

        if not updated:
            return desired

        desired = self.set_desired_targets(updated, source=source)
        return desired

    def _run_loop(self) -> None:
        while not self._loop_stop.is_set():
            started = time.perf_counter()
            try:
                safety = self.safety_snapshot()
                if safety.motion_allowed:
                    state = self.safe_adapter_state()
                    if state.get("connected"):
                        observed = state.get("joints", {})
                        commanded = state.get("commanded_joints", observed)
                        self._state_store.set_observed(observed)
                        self._state_store.set_commanded(commanded)
                        desired = self._apply_controller_intent_to_desired(self._loop_dt)
                        filtered = self._motion_filter.step(desired, commanded, self._loop_dt)
                        self._state_store.set_filtered(filtered)

                        next_targets: dict[str, float] = {}
                        next_sources: dict[str, str] = {}
                        for joint_name, filtered_value in filtered.items():
                            commanded_value = float(commanded.get(joint_name, filtered_value))
                            if abs(float(filtered_value) - commanded_value) >= self._control_epsilon:
                                next_targets[joint_name] = float(filtered_value)
                                next_sources[joint_name] = self.current_joint_source(joint_name)

                        if next_targets:
                            for joint_name, source in next_sources.items():
                                self._adapter.note_command_source([joint_name], source)
                            self._adapter.move_joints(next_targets)
                            self._state_store.set_commanded(next_targets)
                with self._lock:
                    self._last_loop_error = None
            except Exception as exc:
                with self._lock:
                    self._last_loop_error = str(exc)
            finally:
                with self._lock:
                    self._last_loop_at = time.time()
                elapsed = time.perf_counter() - started
                self._loop_stop.wait(max(0.0, self._loop_dt - elapsed))
