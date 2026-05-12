from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from app.robot_arm import JointDefinition


@dataclass(frozen=True)
class ControllerBinding:
    kind: str
    mode: str = "velocity"
    axis: str | None = None
    negative_button: str | None = None
    positive_button: str | None = None
    negative_trigger: str | None = None
    positive_trigger: str | None = None
    scale: float = 1.0
    deadzone: float = 0.0
    curve: float = 1.0
    sign: float = 1.0
    enabled: bool = True


@dataclass(frozen=True)
class JointRuntimeConfig:
    name: str
    label: str
    servo_id: int | None
    minimum: float
    maximum: float
    default: float
    neutral: float
    default_start: float
    inverted: bool
    enabled: bool
    semantic_negative_label: str
    semantic_positive_label: str
    notes: str
    controller_mapping: dict[str, Any] = field(default_factory=dict)
    controller_binding: ControllerBinding | None = None
    max_delta_per_command: float | None = None
    max_degrees_per_second: float | None = None
    controller_max_degrees_per_second: float | None = None


def _binding_from_mapping(mapping: dict[str, Any]) -> ControllerBinding | None:
    controller = mapping.get("controller", {})
    if not isinstance(controller, dict):
        return None
    ps4 = controller.get("ps4", {})
    if not isinstance(ps4, dict):
        return None
    return ControllerBinding(
        kind=str(ps4.get("kind", "axis")),
        mode=str(ps4.get("mode", "velocity")),
        axis=ps4.get("axis"),
        negative_button=ps4.get("negative_button"),
        positive_button=ps4.get("positive_button"),
        negative_trigger=ps4.get("negative_trigger"),
        positive_trigger=ps4.get("positive_trigger"),
        scale=float(ps4.get("scale", 1.0)),
        deadzone=float(ps4.get("deadzone", 0.0)),
        curve=float(ps4.get("curve", 1.0)),
        sign=float(ps4.get("sign", 1.0)),
        enabled=bool(ps4.get("enabled", True)),
    )


def build_joint_runtime_configs(
    config: dict[str, Any],
    joint_definitions: dict[str, JointDefinition],
) -> dict[str, JointRuntimeConfig]:
    servo_map = config.get("robot", {}).get("servo_map", {})
    motion_policy = config.get("robot", {}).get("motion_policy", {})
    teleop = config.get("robot", {}).get("teleop", {})
    joint_configs: dict[str, JointRuntimeConfig] = {}
    for name, definition in joint_definitions.items():
        mapping = servo_map.get(name, {})
        controller_mapping = mapping.get("controller_mapping", {})
        if isinstance(controller_mapping, str):
            controller_mapping = {"physical_ps4": controller_mapping}
        joint_configs[name] = JointRuntimeConfig(
            name=name,
            label=str(mapping.get("label_for_ui", name.replace("_", " ").title())),
            servo_id=mapping.get("servo_id"),
            minimum=float(definition.minimum),
            maximum=float(definition.maximum),
            default=float(definition.default),
            neutral=float(mapping.get("neutral", definition.default)),
            default_start=float(mapping.get("default_start", definition.default)),
            inverted=bool(mapping.get("invert", False)),
            enabled=bool(mapping.get("enabled", True)),
            semantic_negative_label=str(mapping.get("semantic_negative_label", "Min")),
            semantic_positive_label=str(mapping.get("semantic_positive_label", "Max")),
            notes=str(mapping.get("notes", "")),
            controller_mapping=dict(controller_mapping),
            controller_binding=_binding_from_mapping(mapping),
            max_delta_per_command=(
                None
                if mapping.get("max_delta_per_command", motion_policy.get("default_max_delta_per_command")) is None
                else float(mapping.get("max_delta_per_command", motion_policy.get("default_max_delta_per_command")))
            ),
            max_degrees_per_second=(
                None
                if mapping.get("max_degrees_per_second", motion_policy.get("default_max_degrees_per_second")) is None
                else float(mapping.get("max_degrees_per_second", motion_policy.get("default_max_degrees_per_second")))
            ),
            controller_max_degrees_per_second=(
                None
                if mapping.get("controller_max_degrees_per_second", teleop.get("default_controller_max_degrees_per_second")) is None
                else float(mapping.get("controller_max_degrees_per_second", teleop.get("default_controller_max_degrees_per_second")))
            ),
        )
    return joint_configs
