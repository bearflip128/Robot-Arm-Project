"""Config loading and validation.

Fails loudly at startup rather than producing a half-configured arm.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any

import yaml

from .joints import Binding, JointSpec


@dataclass(frozen=True)
class Settings:
    host: str
    port: int
    username: str
    password: str
    secret_key: str
    serial_port: str
    baud: int
    driver: str
    control_hz: float
    intent_timeout: float
    latency_ms: int
    goal_speed: int
    goal_accel: int
    nominal_volts: float
    undervolt_warn: float
    specs: dict[str, JointSpec]
    gamepad: dict[str, Any]
    config_path: Path  # so calibration can write the travel it measures back


def _binding(raw: dict[str, Any] | None) -> Binding:
    raw = raw or {}
    kind = str(raw.get("kind", "none"))
    if kind not in ("axis", "trigger_pair", "button_pair", "none"):
        raise ValueError(f"Unknown binding kind: {kind}")
    return Binding(
        kind=kind,
        axis=raw.get("axis"),
        negative=raw.get("negative"),
        positive=raw.get("positive"),
        deadzone=float(raw.get("deadzone", 0.10)),
        expo=float(raw.get("expo", 1.6)),
        sign=float(raw.get("sign", 1.0)),
    )


def load(path: str | Path) -> Settings:
    config_path = Path(path)
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    motion = data.get("motion", {})
    specs: dict[str, JointSpec] = {}
    for name, raw in data["joints"].items():
        specs[name] = JointSpec(
            name=name,
            label=str(raw.get("label", name.replace("_", " ").title())),
            servo_id=int(raw["servo_id"]),
            lo=float(raw["lo"]),
            hi=float(raw["hi"]),
            home=float(raw["home"]),
            step_lo=int(raw["step_lo"]),
            step_hi=int(raw["step_hi"]),
            invert=bool(raw.get("invert", False)),
            margin_steps=int(raw.get("margin_steps", motion.get("margin_steps", 0))),
            max_vel=float(raw.get("max_vel", motion.get("max_vel", 120.0))),
            max_accel=float(raw.get("max_accel", motion.get("max_accel", 400.0))),
            teleop_vel=float(raw.get("teleop_vel", motion.get("teleop_vel", 90.0))),
            lead_limit=float(raw.get("lead_limit", motion.get("lead_limit", 12.0))),
            binding=_binding(raw.get("binding")),
        )

    if not specs:
        raise ValueError("Config defines no joints.")

    ids = [spec.servo_id for spec in specs.values()]
    if len(set(ids)) != len(ids):
        raise ValueError(f"Duplicate servo_id in joints config: {sorted(ids)}")

    app = data.get("app", {})
    auth = data.get("auth", {})
    robot = data.get("robot", {})
    return Settings(
        host=str(app.get("host", "127.0.0.1")),
        port=int(app.get("port", 7002)),
        # Environment wins over the file so real credentials never need to be
        # written into a config that gets committed. This dashboard is reachable
        # from the internet and drives a physical arm, so the checked-in values
        # are deliberately placeholders.
        username=os.environ.get("ARMCTL_USERNAME") or str(auth.get("username", "admin")),
        password=os.environ.get("ARMCTL_PASSWORD") or str(auth.get("password", "change-me")),
        secret_key=(os.environ.get("ARMCTL_SECRET_KEY")
                    or str(auth.get("secret_key", "change-me-session"))),
        serial_port=str(robot.get("serial_port", "COM4")),
        baud=int(robot.get("baud", 1_000_000)),
        driver=str(robot.get("driver", "feetech")),
        control_hz=float(robot.get("control_hz", 100.0)),
        intent_timeout=float(robot.get("intent_timeout", 0.25)),
        latency_ms=int(robot.get("latency_ms", 8)),
        goal_speed=int(robot.get("goal_speed", 0)),
        goal_accel=int(robot.get("goal_accel", 0)),
        nominal_volts=float(robot.get("nominal_volts", 5.0)),
        undervolt_warn=float(robot.get("undervolt_warn", 4.5)),
        specs=specs,
        gamepad=dict(data.get("gamepad") or {}),
        config_path=config_path,
    )
