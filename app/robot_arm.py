from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


@dataclass(frozen=True)
class JointDefinition:
    name: str
    minimum: float
    maximum: float
    default: float


class RobotArmAdapter:
    def __init__(self, joint_definitions: Dict[str, JointDefinition]) -> None:
        self.joint_definitions = joint_definitions

    def connect(self) -> bool:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError

    def disable_torque(self) -> None:
        raise NotImplementedError

    def get_state(self) -> Dict[str, Any]:
        raise NotImplementedError

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        raise NotImplementedError


class MockRobotArmAdapter(RobotArmAdapter):
    def __init__(self, joint_definitions: Dict[str, JointDefinition]) -> None:
        super().__init__(joint_definitions)
        self.connected = False
        self.joints = {
            name: definition.default for name, definition in joint_definitions.items()
        }

    def connect(self) -> bool:
        self.connected = True
        return True

    def disconnect(self) -> None:
        self.connected = False

    def disable_torque(self) -> None:
        self.connected = False

    def get_state(self) -> Dict[str, Any]:
        return {"connected": self.connected, "joints": dict(self.joints)}

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        for name, value in targets.items():
            if name in self.joints:
                self.joints[name] = value
        return dict(self.joints)

    def reset_to_defaults(self) -> Dict[str, float]:
        self.joints = {
            name: definition.default for name, definition in self.joint_definitions.items()
        }
        return dict(self.joints)


class LeRobotSO100ArmAdapter(RobotArmAdapter):
    def __init__(
        self,
        joint_definitions: Dict[str, JointDefinition],
        robot_config: Dict[str, Any],
    ) -> None:
        super().__init__(joint_definitions)
        self.robot_config = robot_config
        self.robot = None

    def connect(self) -> bool:
        from lerobot.robots.so100_follower.config_so100_follower import (
            SO100FollowerConfig,
        )
        from lerobot.robots.so100_follower.so100_follower import SO100Follower

        config = SO100FollowerConfig(
            port=self.robot_config["port"],
            id=self.robot_config["id"],
            use_degrees=self.robot_config.get("use_degrees", True),
        )
        self.robot = SO100Follower(config)
        self.robot.connect()
        return True

    def disconnect(self) -> None:
        if self.robot is not None:
            disconnect = getattr(self.robot, "disconnect", None)
            if callable(disconnect):
                disconnect()
        self.robot = None

    def disable_torque(self) -> None:
        if self.robot is not None:
            self.robot.bus.disable_torque()

    def get_state(self) -> Dict[str, Any]:
        if self.robot is None:
            joints = {
                name: definition.default
                for name, definition in self.joint_definitions.items()
            }
            return {"connected": False, "joints": joints}

        observation = self.robot.get_observation()
        joints = {}
        for name, definition in self.joint_definitions.items():
            joints[name] = float(observation.get(f"{name}.pos", definition.default))
        return {"connected": True, "joints": joints}

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")
        action = {f"{name}.pos": float(value) for name, value in targets.items()}
        self.robot.send_action(action)
        return self.get_state()["joints"]


def load_config(config_path: str | Path) -> Dict[str, Any]:
    path = Path(config_path)
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def build_joint_definitions(config: Dict[str, Any]) -> Dict[str, JointDefinition]:
    definitions = {}
    for name, values in config["joints"].items():
        definitions[name] = JointDefinition(
            name=name,
            minimum=float(values["min"]),
            maximum=float(values["max"]),
            default=float(values["default"]),
        )
    return definitions


def build_adapter(config: Dict[str, Any]) -> RobotArmAdapter:
    joint_definitions = build_joint_definitions(config)
    driver = config["robot"].get("driver", "mock")

    if driver == "mock":
        return MockRobotArmAdapter(joint_definitions)
    if driver == "lerobot_so100":
        return LeRobotSO100ArmAdapter(joint_definitions, config["robot"])

    raise ValueError(f"Unsupported robot driver: {driver}")


def default_joint_positions(
    joint_definitions: Dict[str, JointDefinition],
) -> Dict[str, float]:
    return {
        name: definition.default for name, definition in joint_definitions.items()
    }
