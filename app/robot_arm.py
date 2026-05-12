from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time
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

    def enable_torque(self) -> None:
        raise NotImplementedError

    def get_state(self) -> Dict[str, Any]:
        raise NotImplementedError

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        raise NotImplementedError

    def set_joint_torque(self, joint_name: str, enabled: bool) -> None:
        raise NotImplementedError

    def get_joint_raw_step(self, joint_name: str) -> int:
        raise NotImplementedError

    def get_diagnostics(self) -> Dict[str, Any]:
        return {}

    def note_command_source(self, joint_names: list[str] | tuple[str, ...], source: str) -> None:
        return None


class MockRobotArmAdapter(RobotArmAdapter):
    def __init__(self, joint_definitions: Dict[str, JointDefinition]) -> None:
        super().__init__(joint_definitions)
        self.connected = False
        self.joints = {
            name: definition.default for name, definition in joint_definitions.items()
        }
        self.commanded_joints = dict(self.joints)
        self.command_sources = {
            name: "system" for name in joint_definitions
        }

    def connect(self) -> bool:
        self.connected = True
        return True

    def disconnect(self) -> None:
        self.connected = False

    def disable_torque(self) -> None:
        self.connected = False

    def enable_torque(self) -> None:
        self.connected = True

    def get_state(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "joints": dict(self.joints),
            "commanded_joints": dict(self.commanded_joints),
        }

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        for name, value in targets.items():
            if name in self.joints:
                self.joints[name] = value
                self.commanded_joints[name] = value
        return dict(self.joints)

    def note_command_source(self, joint_names: list[str] | tuple[str, ...], source: str) -> None:
        for joint_name in joint_names:
            if joint_name in self.command_sources:
                self.command_sources[joint_name] = source

    def reset_to_defaults(self) -> Dict[str, float]:
        self.joints = {
            name: definition.default for name, definition in self.joint_definitions.items()
        }
        self.commanded_joints = dict(self.joints)
        return dict(self.joints)

    def set_joint_torque(self, joint_name: str, enabled: bool) -> None:
        return None

    def get_joint_raw_step(self, joint_name: str) -> int:
        definition = self.joint_definitions[joint_name]
        joint_value = self.joints.get(joint_name, definition.default)
        span = definition.maximum - definition.minimum
        if span <= 0:
            return 0
        normalized = (joint_value - definition.minimum) / span
        normalized = max(0.0, min(1.0, normalized))
        return int(round(normalized * 4095))

    def get_diagnostics(self) -> Dict[str, Any]:
        return {
            "driver": "mock",
            "servo_status": {
                joint_name: {"active_source": source}
                for joint_name, source in self.command_sources.items()
            },
        }


class LeRobotSO100ArmAdapter(RobotArmAdapter):
    def __init__(
        self,
        joint_definitions: Dict[str, JointDefinition],
        robot_config: Dict[str, Any],
    ) -> None:
        super().__init__(joint_definitions)
        self.robot_config = robot_config
        self.robot = None
        self.commanded_joints = {
            name: definition.default for name, definition in joint_definitions.items()
        }
        self.command_sources = {
            name: "system" for name in joint_definitions
        }

    def connect(self) -> bool:
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

        config = SOFollowerRobotConfig(
            port=self.robot_config.get("port") or self.robot_config.get("serial_port"),
            id=self.robot_config.get("lerobot_robot_id") or self.robot_config.get("id"),
            use_degrees=self.robot_config.get("use_degrees", True),
        )
        self.robot = SOFollower(config)
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

    def enable_torque(self) -> None:
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")
        enable_torque = getattr(self.robot.bus, "enable_torque", None)
        if callable(enable_torque):
            enable_torque()

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
        return {
            "connected": True,
            "joints": joints,
            "commanded_joints": dict(self.commanded_joints),
        }

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")
        action = {f"{name}.pos": float(value) for name, value in targets.items()}
        for name, value in targets.items():
            if name in self.commanded_joints:
                self.commanded_joints[name] = float(value)
        self.robot.send_action(action)
        return self.get_state()["joints"]

    def note_command_source(self, joint_names: list[str] | tuple[str, ...], source: str) -> None:
        for joint_name in joint_names:
            if joint_name in self.command_sources:
                self.command_sources[joint_name] = source

    def set_joint_torque(self, joint_name: str, enabled: bool) -> None:
        raise RuntimeError("Per-joint torque control is not supported for this driver.")

    def get_joint_raw_step(self, joint_name: str) -> int:
        raise RuntimeError("Raw step reads are not supported for this driver.")

    def get_diagnostics(self) -> Dict[str, Any]:
        return {
            "driver": "lerobot_so100",
            "servo_status": {
                joint_name: {"active_source": source}
                for joint_name, source in self.command_sources.items()
            },
        }


class SCServoSingleAdapter(RobotArmAdapter):
    def __init__(
        self,
        joint_definitions: Dict[str, JointDefinition],
        robot_config: Dict[str, Any],
    ) -> None:
        super().__init__(joint_definitions)
        self.robot_config = robot_config
        self.port_handler = None
        self.packet_handler = None
        self.connected = False
        self.joint_name = robot_config.get("joint_name", "shoulder_pan")
        if self.joint_name not in joint_definitions:
            raise ValueError(f"Unknown joint_name for scservo_single: {self.joint_name}")

        self.servo_id = int(robot_config.get("servo_id", 1))
        self.serial_port = str(robot_config.get("serial_port", "COM4"))
        self.baud_rate = int(robot_config.get("baud_rate", 1_000_000))
        self.command_speed = int(robot_config.get("command_speed", 180))
        self.command_acc = int(robot_config.get("command_acc", 30))
        self.read_delay_s = float(robot_config.get("read_delay_s", 0.15))
        self.servo_min_step = int(robot_config.get("servo_min_step", 0))
        self.servo_max_step = int(robot_config.get("servo_max_step", 4095))
        self.cached_joints = {
            name: definition.default for name, definition in joint_definitions.items()
        }
        self.commanded_joints = dict(self.cached_joints)
        self.last_error: str | None = None
        self.command_sources = {
            name: "system" for name in joint_definitions
        }

    def connect(self) -> bool:
        if self.connected and self.port_handler is not None and self.packet_handler is not None:
            return True

        from scservo_sdk import PortHandler, sms_sts

        port_handler = PortHandler(self.serial_port)
        if not port_handler.openPort():
            raise RuntimeError(f"Unable to open serial port {self.serial_port}.")
        if not port_handler.setBaudRate(self.baud_rate):
            port_handler.closePort()
            raise RuntimeError(
                f"Unable to set baud rate {self.baud_rate} on {self.serial_port}."
            )

        self.port_handler = port_handler
        self.packet_handler = sms_sts(port_handler)
        self.connected = True
        self.cached_joints[self.joint_name] = self._read_joint_position()
        self.commanded_joints[self.joint_name] = self.cached_joints[self.joint_name]
        return True

    def disconnect(self) -> None:
        if self.port_handler is not None:
            self.port_handler.closePort()
        self.port_handler = None
        self.packet_handler = None
        self.connected = False

    def disable_torque(self) -> None:
        if not self.connected or self.packet_handler is None:
            return
        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        _, result, error = self.packet_handler.write1ByteTxRx(
            self.servo_id, SMS_STS_TORQUE_ENABLE, 0
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Failed to disable torque for servo {self.servo_id} on {self.serial_port}."
            )

    def enable_torque(self) -> None:
        if not self.connected or self.packet_handler is None:
            return
        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        result, error = self.packet_handler.write1ByteTxRx(
            self.servo_id, SMS_STS_TORQUE_ENABLE, 1
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Failed to enable torque for servo {self.servo_id} on {self.serial_port}."
            )

    def get_state(self) -> Dict[str, Any]:
        joints = dict(self.cached_joints)
        if self.connected:
            joints[self.joint_name] = self._read_joint_position()
            self.cached_joints[self.joint_name] = joints[self.joint_name]
        return {
            "connected": self.connected,
            "joints": joints,
            "commanded_joints": dict(self.commanded_joints),
        }

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        for name, value in targets.items():
            if name != self.joint_name and name in self.cached_joints:
                self.cached_joints[name] = float(value)
                self.commanded_joints[name] = float(value)

        if self.joint_name not in targets:
            return self.get_state()["joints"]

        if not self.connected or self.packet_handler is None:
            raise RuntimeError("Servo is not connected.")

        from scservo_sdk import COMM_SUCCESS

        target_step = self._joint_to_step(float(targets[self.joint_name]))
        self.commanded_joints[self.joint_name] = float(targets[self.joint_name])
        result, error = self.packet_handler.WritePosEx(
            self.servo_id,
            target_step,
            self.command_speed,
            self.command_acc,
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Failed to move servo {self.servo_id} on {self.serial_port}."
            )

        time.sleep(self.read_delay_s)
        self.cached_joints[self.joint_name] = self._read_joint_position()
        return dict(self.cached_joints)

    def _read_joint_position(self) -> float:
        if not self.connected or self.packet_handler is None:
            return self.cached_joints[self.joint_name]

        from scservo_sdk import COMM_SUCCESS

        position, result, error = self.packet_handler.ReadPos(self.servo_id)
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Failed to read servo {self.servo_id} on {self.serial_port}."
            )
        return self._step_to_joint(int(position))

    def _joint_to_step(self, joint_value: float) -> int:
        definition = self.joint_definitions[self.joint_name]
        joint_span = definition.maximum - definition.minimum
        servo_span = self.servo_max_step - self.servo_min_step
        if joint_span <= 0 or servo_span <= 0:
            raise RuntimeError("Invalid joint or servo range configuration.")
        normalized = (joint_value - definition.minimum) / joint_span
        normalized = max(0.0, min(1.0, normalized))
        return int(round(self.servo_min_step + normalized * servo_span))

    def _step_to_joint(self, servo_step: int) -> float:
        definition = self.joint_definitions[self.joint_name]
        servo_span = self.servo_max_step - self.servo_min_step
        if servo_span <= 0:
            raise RuntimeError("Invalid servo range configuration.")
        normalized = (servo_step - self.servo_min_step) / servo_span
        normalized = max(0.0, min(1.0, normalized))
        return definition.minimum + normalized * (definition.maximum - definition.minimum)

    def set_joint_torque(self, joint_name: str, enabled: bool) -> None:
        if joint_name != self.joint_name:
            raise RuntimeError(f"Joint {joint_name} is not controlled by this adapter.")
        if not self.connected or self.packet_handler is None:
            raise RuntimeError("Servo is not connected.")

        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        result, error = self.packet_handler.write1ByteTxRx(
            self.servo_id, SMS_STS_TORQUE_ENABLE, 1 if enabled else 0
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Failed to {'enable' if enabled else 'disable'} torque for servo {self.servo_id} on {self.serial_port}."
            )

    def get_joint_raw_step(self, joint_name: str) -> int:
        if joint_name != self.joint_name:
            raise RuntimeError(f"Joint {joint_name} is not controlled by this adapter.")
        if not self.connected or self.packet_handler is None:
            raise RuntimeError("Servo is not connected.")

        from scservo_sdk import COMM_SUCCESS

        position, result, error = self.packet_handler.ReadPos(self.servo_id)
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"Failed to read raw step for servo {self.servo_id} on {self.serial_port}."
            )
        return int(position)

    def get_diagnostics(self) -> Dict[str, Any]:
        return {
            "driver": "scservo_single",
            "servo_status": {
                self.joint_name: {
                    "servo_id": self.servo_id,
                    "last_error": self.last_error,
                    "active_source": self.command_sources.get(self.joint_name, "system"),
                }
            },
        }

    def note_command_source(self, joint_names: list[str] | tuple[str, ...], source: str) -> None:
        for joint_name in joint_names:
            if joint_name in self.command_sources:
                self.command_sources[joint_name] = source


class SCServoBusAdapter(RobotArmAdapter):
    def __init__(
        self,
        joint_definitions: Dict[str, JointDefinition],
        robot_config: Dict[str, Any],
    ) -> None:
        super().__init__(joint_definitions)
        self.robot_config = robot_config
        self.port_handler = None
        self.packet_handler = None
        self.connected = False
        self.serial_port = str(robot_config.get("serial_port", "COM4"))
        self.baud_rate = int(robot_config.get("baud_rate", 1_000_000))
        self.command_speed = int(robot_config.get("command_speed", 180))
        self.command_acc = int(robot_config.get("command_acc", 30))
        self.read_delay_s = float(robot_config.get("read_delay_s", 0.15))
        self.read_retries = int(robot_config.get("read_retries", 3))
        self.write_retries = int(robot_config.get("write_retries", 2))
        motion_policy = robot_config.get("motion_policy", {})
        self.default_max_delta_per_command = motion_policy.get(
            "default_max_delta_per_command"
        )
        self.default_max_degrees_per_second = motion_policy.get(
            "default_max_degrees_per_second"
        )
        self._io_lock = threading.RLock()
        self.default_servo_min_step = int(robot_config.get("servo_min_step", 0))
        self.default_servo_max_step = int(robot_config.get("servo_max_step", 4095))
        self.cached_joints = {
            name: definition.default for name, definition in joint_definitions.items()
        }
        self.commanded_joints = dict(self.cached_joints)
        self.servo_status: Dict[str, Dict[str, Any]] = {}

        raw_map = robot_config.get("servo_map", {})
        if not raw_map:
            raise ValueError("scservo_bus requires a servo_map in robot config.")

        self.servo_map: Dict[str, Dict[str, Any]] = {}
        for joint_name, mapping in raw_map.items():
            if joint_name not in joint_definitions:
                raise ValueError(f"Unknown joint in servo_map: {joint_name}")
            self.servo_map[joint_name] = {
                "servo_id": int(mapping["servo_id"]),
                "servo_min_step": int(mapping.get("servo_min_step", self.default_servo_min_step)),
                "servo_max_step": int(mapping.get("servo_max_step", self.default_servo_max_step)),
                "invert": bool(mapping.get("invert", False)),
                "command_speed": int(mapping.get("command_speed", self.command_speed)),
                "command_acc": int(mapping.get("command_acc", self.command_acc)),
                "max_delta_per_command": mapping.get(
                    "max_delta_per_command", self.default_max_delta_per_command
                ),
                "max_degrees_per_second": mapping.get(
                    "max_degrees_per_second", self.default_max_degrees_per_second
                ),
            }
            self.servo_status[joint_name] = {
                "servo_id": int(mapping["servo_id"]),
                "torque_enabled": None,
                "last_error": None,
                "raw_step": None,
                "actual_joint": self.cached_joints[joint_name],
                "commanded_joint": self.commanded_joints[joint_name],
                "last_sent_joint": self.commanded_joints[joint_name],
                "pending_joint": None,
                "clamped_by_limit": False,
                "rate_limited": False,
                "filtered_reasons": [],
                "last_read_at": None,
                "last_command_at": None,
                "active_source": "system",
                "truth_confidence": "low",
                "unreadable": True,
            }

    def connect(self) -> bool:
        with self._io_lock:
            if self.connected and self.port_handler is not None and self.packet_handler is not None:
                return True

            from scservo_sdk import PortHandler, sms_sts

            port_handler = PortHandler(self.serial_port)
            if not port_handler.openPort():
                raise RuntimeError(f"Unable to open serial port {self.serial_port}.")
            if not port_handler.setBaudRate(self.baud_rate):
                port_handler.closePort()
                raise RuntimeError(
                    f"Unable to set baud rate {self.baud_rate} on {self.serial_port}."
                )

            self.port_handler = port_handler
            self.packet_handler = sms_sts(port_handler)
            self.connected = True
            self._enable_torque_all()
            self.cached_joints.update(self._read_all_joint_positions())
            if not self._has_any_readable_servo():
                self.disconnect()
                raise RuntimeError(
                    f"Connected to {self.serial_port}, but could not read any configured servos. "
                    "Check power, COM port, and bus wiring."
                )
            self.commanded_joints.update(self.cached_joints)
            return True

    def disconnect(self) -> None:
        with self._io_lock:
            if self.port_handler is not None:
                self.port_handler.closePort()
            self.port_handler = None
            self.packet_handler = None
            self.connected = False
            for status in self.servo_status.values():
                status["torque_enabled"] = None

    def disable_torque(self) -> None:
        with self._io_lock:
            if not self.connected or self.packet_handler is None:
                return
            from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

            for joint_name, mapping in self.servo_map.items():
                result, error = self.packet_handler.write1ByteTxRx(
                    mapping["servo_id"], SMS_STS_TORQUE_ENABLE, 0
                )
                if result != COMM_SUCCESS or error != 0:
                    self.servo_status[joint_name]["last_error"] = f"disable_torque:{error}"
                    raise RuntimeError(
                        f"Failed to disable torque for servo {mapping['servo_id']} on {self.serial_port}."
                    )
                self.servo_status[joint_name]["torque_enabled"] = False
                self.servo_status[joint_name]["last_error"] = None

    def enable_torque(self) -> None:
        with self._io_lock:
            if not self.connected or self.packet_handler is None:
                return
            self._enable_torque_all()

    def _enable_torque_all(self) -> None:
        if not self.connected or self.packet_handler is None:
            return

        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        for joint_name, mapping in self.servo_map.items():
            result, error = self.packet_handler.write1ByteTxRx(
                mapping["servo_id"], SMS_STS_TORQUE_ENABLE, 1
            )
            if result != COMM_SUCCESS or error != 0:
                self.servo_status[joint_name]["torque_enabled"] = False
                self.servo_status[joint_name]["last_error"] = f"enable_torque:{error}"
                continue
            self.servo_status[joint_name]["torque_enabled"] = True
            self.servo_status[joint_name]["last_error"] = None

    def set_joint_torque(self, joint_name: str, enabled: bool) -> None:
        with self._io_lock:
            if not self.connected or self.packet_handler is None:
                raise RuntimeError("Servo bus is not connected.")
            if joint_name not in self.servo_map:
                raise RuntimeError(f"Joint {joint_name} is not mapped to a servo.")

            from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

            servo_id = self.servo_map[joint_name]["servo_id"]
            result, error = self.packet_handler.write1ByteTxRx(
                servo_id, SMS_STS_TORQUE_ENABLE, 1 if enabled else 0
            )
            if result != COMM_SUCCESS or error != 0:
                self.servo_status[joint_name]["last_error"] = (
                    f"{'enable' if enabled else 'disable'}_torque:{error}"
                )
                raise RuntimeError(
                    f"Failed to {'enable' if enabled else 'disable'} torque for servo {servo_id} on {self.serial_port}."
                )
            self.servo_status[joint_name]["torque_enabled"] = enabled
            self.servo_status[joint_name]["last_error"] = None

    def get_joint_raw_step(self, joint_name: str) -> int:
        with self._io_lock:
            if not self.connected or self.packet_handler is None:
                raise RuntimeError("Servo bus is not connected.")
            if joint_name not in self.servo_map:
                raise RuntimeError(f"Joint {joint_name} is not mapped to a servo.")

            servo_id = self.servo_map[joint_name]["servo_id"]
            position = self._read_servo_position(servo_id)
            if position is None:
                raise RuntimeError(f"Failed to read raw step for servo {servo_id} on {self.serial_port}.")
            return int(position)

    def get_diagnostics(self) -> Dict[str, Any]:
        return {
            "driver": "scservo_bus",
            "servo_status": self.servo_status,
        }

    def get_state(self) -> Dict[str, Any]:
        with self._io_lock:
            joints = dict(self.cached_joints)
            if self.connected:
                joints.update(self._read_all_joint_positions())
                self.cached_joints.update(joints)
            bus_healthy = self._has_any_readable_servo()
            state: Dict[str, Any] = {
                "connected": self.connected and bus_healthy,
                "joints": joints,
                "commanded_joints": dict(self.commanded_joints),
            }
            if self.connected and not bus_healthy:
                state["hardware_error"] = (
                    f"Connected to {self.serial_port}, but no configured servos are readable."
                )
            return state

    def move_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        with self._io_lock:
            if not self.connected or self.packet_handler is None:
                raise RuntimeError("Servo bus is not connected.")
            if not self._has_any_readable_servo():
                raise RuntimeError(
                    f"Connected to {self.serial_port}, but no configured servos are readable."
                )

            for joint_name, value in targets.items():
                if joint_name not in self.servo_map:
                    if joint_name in self.cached_joints:
                        self.cached_joints[joint_name] = float(value)
                        self.commanded_joints[joint_name] = float(value)
                    continue

                mapping = self.servo_map[joint_name]
                filtered_value = self._apply_motion_policy(joint_name, float(value))
                target_step = self._joint_to_step(joint_name, filtered_value)
                self.commanded_joints[joint_name] = float(filtered_value)
                self.servo_status[joint_name]["commanded_joint"] = float(value)
                self.servo_status[joint_name]["last_sent_joint"] = float(filtered_value)
                self.servo_status[joint_name]["pending_joint"] = float(filtered_value)
                self.servo_status[joint_name]["last_command_at"] = time.time()
                if not self._write_joint_target(
                    mapping["servo_id"],
                    target_step,
                    mapping["command_speed"],
                    mapping["command_acc"],
                ):
                    self.servo_status[joint_name]["last_error"] = "move_failed"
                    self.servo_status[joint_name]["truth_confidence"] = "low"
                    raise RuntimeError(
                        f"Failed to move servo {mapping['servo_id']} for joint {joint_name}."
                    )
                self.servo_status[joint_name]["last_error"] = None

            time.sleep(self.read_delay_s)
            self.cached_joints.update(self._read_all_joint_positions())
            return dict(self.cached_joints)

    def _read_all_joint_positions(self) -> Dict[str, float]:
        if not self.connected or self.packet_handler is None:
            return dict(self.cached_joints)

        joint_positions: Dict[str, float] = {}
        for joint_name, mapping in self.servo_map.items():
            position = self._read_servo_position(mapping["servo_id"])
            if position is None:
                self.servo_status[joint_name]["unreadable"] = True
                self.servo_status[joint_name]["truth_confidence"] = (
                    "medium" if joint_name in self.cached_joints else "low"
                )
                self.servo_status[joint_name]["last_error"] = "read_failed"
                if joint_name in self.cached_joints:
                    joint_positions[joint_name] = self.cached_joints[joint_name]
                    continue
                raise RuntimeError(
                    f"Failed to read servo {mapping['servo_id']} for joint {joint_name}."
                )
            self.servo_status[joint_name]["raw_step"] = int(position)
            self.servo_status[joint_name]["last_error"] = None
            self.servo_status[joint_name]["last_read_at"] = time.time()
            self.servo_status[joint_name]["truth_confidence"] = "high"
            self.servo_status[joint_name]["unreadable"] = False
            joint_positions[joint_name] = self._step_to_joint(joint_name, int(position))
            self.servo_status[joint_name]["actual_joint"] = joint_positions[joint_name]
            self.servo_status[joint_name]["commanded_joint"] = self.commanded_joints.get(
                joint_name, joint_positions[joint_name]
            )
            if self.servo_status[joint_name]["last_command_at"] is None:
                self.servo_status[joint_name]["last_sent_joint"] = self.commanded_joints.get(
                    joint_name, joint_positions[joint_name]
                )
            self.servo_status[joint_name]["pending_joint"] = None
        return joint_positions

    def _write_joint_target(
        self,
        servo_id: int,
        target_step: int,
        command_speed: int,
        command_acc: int,
    ) -> bool:
        if self.packet_handler is None:
            return False

        from scservo_sdk import COMM_SUCCESS

        for attempt in range(max(1, self.write_retries)):
            result, error = self.packet_handler.WritePosEx(
                servo_id,
                target_step,
                command_speed,
                command_acc,
            )
            if result == COMM_SUCCESS and error == 0:
                return True
            time.sleep(0.03 * (attempt + 1))
        return False

    def _read_servo_position(self, servo_id: int) -> int | None:
        if self.packet_handler is None:
            return None

        from scservo_sdk import COMM_SUCCESS

        for attempt in range(max(1, self.read_retries)):
            position, result, error = self.packet_handler.ReadPos(servo_id)
            if result == COMM_SUCCESS and error == 0:
                return int(position)
            time.sleep(0.02 * (attempt + 1))
        return None

    def _has_any_readable_servo(self) -> bool:
        return any(
            status.get("raw_step") is not None and status.get("last_error") in (None, "")
            for status in self.servo_status.values()
        )

    def _apply_motion_policy(self, joint_name: str, requested_value: float) -> float:
        definition = self.joint_definitions[joint_name]
        mapping = self.servo_map[joint_name]
        now = time.time()
        filtered_value = float(requested_value)
        filtered_reasons: list[str] = []

        if filtered_value < definition.minimum or filtered_value > definition.maximum:
            filtered_value = max(definition.minimum, min(definition.maximum, filtered_value))
            filtered_reasons.append("joint_limit")

        baseline = float(self.commanded_joints.get(joint_name, self.cached_joints[joint_name]))

        max_delta = mapping.get("max_delta_per_command")
        if max_delta is not None:
            max_delta_value = float(max_delta)
            if max_delta_value > 0:
                delta = filtered_value - baseline
                if abs(delta) > max_delta_value:
                    filtered_value = baseline + max_delta_value * (1 if delta > 0 else -1)
                    filtered_reasons.append("max_delta")

        max_rate = mapping.get("max_degrees_per_second")
        last_command_at = self.servo_status[joint_name].get("last_command_at")
        if max_rate is not None and last_command_at is not None:
            elapsed = max(0.02, now - float(last_command_at))
            allowed_delta = float(max_rate) * elapsed
            delta = filtered_value - baseline
            if allowed_delta > 0 and abs(delta) > allowed_delta:
                filtered_value = baseline + allowed_delta * (1 if delta > 0 else -1)
                filtered_reasons.append("max_rate")

        self.servo_status[joint_name]["clamped_by_limit"] = "joint_limit" in filtered_reasons
        self.servo_status[joint_name]["rate_limited"] = any(
            reason in filtered_reasons for reason in ("max_delta", "max_rate")
        )
        self.servo_status[joint_name]["filtered_reasons"] = filtered_reasons
        return max(definition.minimum, min(definition.maximum, float(filtered_value)))

    def _joint_to_step(self, joint_name: str, joint_value: float) -> int:
        definition = self.joint_definitions[joint_name]
        mapping = self.servo_map[joint_name]
        joint_span = definition.maximum - definition.minimum
        servo_span = mapping["servo_max_step"] - mapping["servo_min_step"]
        if joint_span <= 0 or servo_span <= 0:
            raise RuntimeError("Invalid joint or servo range configuration.")
        normalized = (joint_value - definition.minimum) / joint_span
        normalized = max(0.0, min(1.0, normalized))
        if mapping["invert"]:
            normalized = 1.0 - normalized
        return int(round(mapping["servo_min_step"] + normalized * servo_span))

    def _step_to_joint(self, joint_name: str, servo_step: int) -> float:
        definition = self.joint_definitions[joint_name]
        mapping = self.servo_map[joint_name]
        servo_span = mapping["servo_max_step"] - mapping["servo_min_step"]
        if servo_span <= 0:
            raise RuntimeError("Invalid servo range configuration.")
        normalized = (servo_step - mapping["servo_min_step"]) / servo_span
        normalized = max(0.0, min(1.0, normalized))
        if mapping["invert"]:
            normalized = 1.0 - normalized
        return definition.minimum + normalized * (definition.maximum - definition.minimum)

    def note_command_source(self, joint_names: list[str] | tuple[str, ...], source: str) -> None:
        with self._io_lock:
            for joint_name in joint_names:
                if joint_name in self.servo_status:
                    self.servo_status[joint_name]["active_source"] = source


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
    if driver in {"lerobot_so100", "lerobot_so101", "lerobot_so_follower"}:
        return LeRobotSO100ArmAdapter(joint_definitions, config["robot"])
    if driver == "scservo_single":
        return SCServoSingleAdapter(joint_definitions, config["robot"])
    if driver == "scservo_bus":
        return SCServoBusAdapter(joint_definitions, config["robot"])

    raise ValueError(f"Unsupported robot driver: {driver}")


def default_joint_positions(
    joint_definitions: Dict[str, JointDefinition],
) -> Dict[str, float]:
    return {
        name: definition.default for name, definition in joint_definitions.items()
    }
